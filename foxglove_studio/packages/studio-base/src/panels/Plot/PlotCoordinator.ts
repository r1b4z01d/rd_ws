// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import EventEmitter from "eventemitter3";

import { debouncePromise } from "@foxglove/den/async";
import { filterMap } from "@foxglove/den/collection";
import { parseMessagePath } from "@foxglove/message-path";
import { toSec, subtract as subtractTime } from "@foxglove/rostime";
import { Immutable, Time } from "@foxglove/studio";
import { simpleGetMessagePathDataItems } from "@foxglove/studio-base/components/MessagePathSyntax/simpleGetMessagePathDataItems";
import { stringifyMessagePath } from "@foxglove/studio-base/components/MessagePathSyntax/stringifyRosPath";
import { fillInGlobalVariablesInPath } from "@foxglove/studio-base/components/MessagePathSyntax/useCachedGetMessagePathDataItems";
import { Bounds1D } from "@foxglove/studio-base/components/TimeBasedChart/types";
import { GlobalVariables } from "@foxglove/studio-base/hooks/useGlobalVariables";
import { MessageBlock, PlayerState } from "@foxglove/studio-base/players/types";
import { Bounds } from "@foxglove/studio-base/types/Bounds";
import delay from "@foxglove/studio-base/util/delay";
import { getContrastColor, getLineColor } from "@foxglove/studio-base/util/plotColors";

import { Dataset, InteractionEvent, Scale, UpdateAction } from "./ChartRenderer";
import { OffscreenCanvasRenderer } from "./OffscreenCanvasRenderer";
import {
  CsvDataset,
  IDatasetsBuilder,
  SeriesConfigKey,
  SeriesItem,
  Viewport,
} from "./builders/IDatasetsBuilder";
import { isReferenceLinePlotPathType, PlotConfig } from "./config";

type EventTypes = {
  timeseriesBounds(bounds: Immutable<Bounds1D>): void;

  /** X scale changed. */
  xScaleChanged(scale: Scale | undefined): void;

  /** Current values changed (for displaying in the legend) */
  currentValuesChanged(values: readonly unknown[]): void;

  /** Paths with mismatched data lengths were detected */
  pathsWithMismatchedDataLengthsChanged(pathsWithMismatchedDataLengths: string[]): void;

  /** Rendering updated the viewport. `canReset` is true if the viewport can be reset. */
  viewportChange(canReset: boolean): void;
};

const replaceUndefinedWithEmptyDataset = (dataset: Dataset | undefined) => dataset ?? { data: [] };

/**
 * PlotCoordinator interfaces commands and updates between the dataset builder and the chart
 * renderer.
 */
export class PlotCoordinator extends EventEmitter<EventTypes> {
  #renderer: OffscreenCanvasRenderer;
  #datasetsBuilder: IDatasetsBuilder;

  #configBounds: { x: Partial<Bounds1D>; y: Partial<Bounds1D> } = {
    x: {},
    y: {},
  };

  #globalBounds?: Immutable<Partial<Bounds1D>>;
  #datasetRange?: Bounds1D;
  #followRange?: number;
  #interactionBounds?: Bounds;

  #lastSeekTime = NaN;

  /** Normalized series from latest config */
  #series: Immutable<SeriesItem[]> = [];
  /** Current value for each series to show in the legend */
  #currentValuesByConfigIndex: unknown[] = [];

  /** Flag indicating that new Y bounds should be sent to the renderer because the bounds have been reset */
  #shouldResetY = false;

  #updateAction: UpdateAction = { type: "update" };

  #isTimeseriesPlot: boolean = false;
  #currentSeconds?: number;

  #viewport: Viewport = {
    size: { width: 0, height: 0 },
    bounds: { x: undefined, y: undefined },
  };

  #latestXScale?: Scale;

  #queueDispatchRender = debouncePromise(this.#dispatchRender.bind(this));
  #queueDispatchDatasets = debouncePromise(this.#dispatchDatasets.bind(this));
  #queueBlocks = debouncePromise(this.#dispatchBlocks.bind(this));

  #destroyed = false;

  #latestBlocks?: Immutable<(MessageBlock | undefined)[]>;

  public constructor(renderer: OffscreenCanvasRenderer, builder: IDatasetsBuilder) {
    super();

    this.#renderer = renderer;
    this.#datasetsBuilder = builder;
  }

  /** Stop the coordinator from sending any future updates to the renderer. */
  public destroy(): void {
    this.#destroyed = true;
  }

  public handlePlayerState(state: Immutable<PlayerState>): void {
    if (this.#isDestroyed()) {
      return;
    }
    const activeData = state.activeData;
    if (!activeData) {
      return;
    }

    const { messages, lastSeekTime, currentTime, startTime } = activeData;

    if (this.#isTimeseriesPlot) {
      const secondsSinceStart = toSec(subtractTime(currentTime, startTime));
      this.#currentSeconds = secondsSinceStart;
    }

    const datasetsRange = this.#datasetsBuilder.handlePlayerState(state);

    const blocks = state.progress.messageCache?.blocks;
    if (blocks && this.#datasetsBuilder.handleBlocks) {
      this.#latestBlocks = blocks;
      this.#queueBlocks(activeData.startTime, blocks);
    }

    if (lastSeekTime !== this.#lastSeekTime) {
      this.#currentValuesByConfigIndex = [];
      this.#lastSeekTime = lastSeekTime;
    }

    for (const seriesItem of this.#series) {
      if (seriesItem.timestampMethod === "headerStamp") {
        // We currently do not support showing current values in the legend for header.stamp mode,
        // which would require keeping a buffer of messages to sort (currently done in
        // TimestampDatasetsBuilderImpl)
        continue;
      }
      for (let i = messages.length - 1; i >= 0; --i) {
        const msgEvent = messages[i]!;
        if (msgEvent.topic !== seriesItem.parsed.topicName) {
          continue;
        }
        const items = simpleGetMessagePathDataItems(msgEvent, seriesItem.parsed);
        if (items.length > 0) {
          this.#currentValuesByConfigIndex[seriesItem.configIndex] = items[items.length - 1];
          break;
        }
      }
    }

    this.emit("currentValuesChanged", this.#currentValuesByConfigIndex);

    this.#datasetRange = datasetsRange;
    this.#queueDispatchRender();
  }

  public handleConfig(
    config: Immutable<PlotConfig>,
    colorScheme: "light" | "dark",
    globalVariables: GlobalVariables,
  ): void {
    if (this.#isDestroyed()) {
      return;
    }
    this.#isTimeseriesPlot = config.xAxisVal === "timestamp";
    if (!this.#isTimeseriesPlot) {
      this.#currentSeconds = undefined;
    }
    this.#followRange = config.followingViewWidth;

    const newConfigBounds = {
      x: {
        max: config.maxXValue,
        min: config.minXValue,
      },
      y: {
        max:
          config.maxYValue == undefined ||
          (typeof config.maxYValue === "string" && config.maxYValue.length === 0)
            ? undefined
            : +config.maxYValue,
        min:
          config.minYValue == undefined ||
          (typeof config.minYValue === "string" && config.minYValue.length === 0)
            ? undefined
            : +config.minYValue,
      },
    };
    const configYBoundsChanged =
      this.#configBounds.y.min !== newConfigBounds.y.min ||
      this.#configBounds.y.max !== newConfigBounds.y.max;
    this.#configBounds = newConfigBounds;

    const referenceLines = filterMap(config.paths, (path, idx) => {
      if (!path.enabled || !isReferenceLinePlotPathType(path)) {
        return;
      }

      const value = +path.value;
      if (isNaN(value)) {
        return;
      }

      return {
        color: getLineColor(path.color, idx),
        value,
      };
    });

    this.#updateAction.showXAxisLabels = config.showXAxisLabels;
    this.#updateAction.showYAxisLabels = config.showYAxisLabels;
    this.#updateAction.referenceLines = referenceLines;

    if (configYBoundsChanged) {
      // Config changes to yBounds always takes precedence over user interaction changes like pan/zoom
      this.#updateAction.yBounds = this.#configBounds.y;
    }

    const newCurrentValuesByConfigIndex: unknown[] = [];
    this.#series = filterMap(config.paths, (path, idx): Immutable<SeriesItem> | undefined => {
      if (isReferenceLinePlotPathType(path)) {
        return;
      }

      const parsed = parseMessagePath(path.value);
      if (!parsed) {
        return;
      }

      const filledParsed = fillInGlobalVariablesInPath(parsed, globalVariables);

      // When global variables change the path.value is still the original value with the variable
      // names But we need to consider this as a new series (new block cursor) so we compute new
      // values when variables cause the resolved path value to update.
      //
      // We also want to re-compute values when the timestamp method changes. So we use a _key_ that
      // is the filled path and the timestamp method. If either change, we consider this a new
      // series.
      //
      // This key lets us treat series with the same name but different timestamp methods as distinct
      // using a key instead of the path index lets us preserve loaded data when a path is removed
      const key = `${path.timestampMethod}:${stringifyMessagePath(
        filledParsed,
      )}` as SeriesConfigKey;

      // Keep current values for paths that match existing ones
      const existingSeries = this.#series.find((series) => series.key === key);
      if (existingSeries != undefined) {
        newCurrentValuesByConfigIndex[idx] =
          this.#currentValuesByConfigIndex[existingSeries.configIndex];
      }

      const color = getLineColor(path.color, idx);
      return {
        key,
        configIndex: idx,
        messagePath: path.value,
        parsed: filledParsed,
        color,
        contrastColor: getContrastColor(colorScheme, color),
        lineSize: path.lineSize ?? 1.0,
        timestampMethod: path.timestampMethod,
        showLine: path.showLine ?? true,
        enabled: path.enabled,
      };
    });

    this.#datasetsBuilder.setSeries(this.#series);

    this.#currentValuesByConfigIndex = newCurrentValuesByConfigIndex;
    this.emit("currentValuesChanged", this.#currentValuesByConfigIndex);

    this.#queueDispatchRender();
  }

  public setGlobalBounds(bounds: Immutable<Bounds1D> | undefined): void {
    this.#globalBounds = bounds;
    this.#interactionBounds = undefined;
    if (bounds == undefined) {
      // This happens when "reset view" is clicked in a different panel or otherwise cleared the global bounds
      this.#shouldResetY = true;
    }
    this.#queueDispatchRender();
  }

  public setZoomMode(mode: "x" | "xy" | "y"): void {
    this.#updateAction.zoomMode = mode;
    this.#queueDispatchRender();
  }

  public resetBounds(): void {
    this.#interactionBounds = undefined;
    this.#globalBounds = undefined;
    this.#shouldResetY = true;
    this.#queueDispatchRender();
  }

  public setSize(size: { width: number; height: number }): void {
    this.#viewport.size = size;
    this.#updateAction.size = size;
    this.#queueDispatchRender();
  }

  public addInteractionEvent(ev: InteractionEvent): void {
    if (!this.#updateAction.interactionEvents) {
      this.#updateAction.interactionEvents = [];
    }
    this.#updateAction.interactionEvents.push(ev);
    this.#queueDispatchRender();
  }

  /** Get the plot x value at the canvas pixel x location */
  public getXValueAtPixel(pixelX: number): number {
    if (!this.#latestXScale) {
      return -1;
    }

    const pixelRange = this.#latestXScale.right - this.#latestXScale.left;
    if (pixelRange <= 0) {
      return -1;
    }

    // Linear interpolation to place the pixelX value within min/max
    return (
      this.#latestXScale.min +
      ((pixelX - this.#latestXScale.left) / pixelRange) *
        (this.#latestXScale.max - this.#latestXScale.min)
    );
  }

  /** Get the entire data for all series */
  public async getCsvData(): Promise<CsvDataset[]> {
    if (this.#isDestroyed()) {
      return [];
    }
    return await this.#datasetsBuilder.getCsvData();
  }

  /**
   * Return true if the plot viewport has deviated from the config or dataset bounds and can reset
   */
  #canReset(): boolean {
    if (this.#interactionBounds) {
      return true;
    }

    if (this.#globalBounds) {
      const resetBounds = this.#getXResetBounds();
      return (
        this.#globalBounds.min !== resetBounds.min || this.#globalBounds.max !== resetBounds.max
      );
    }

    return false;
  }

  /**
   * Get the xBounds if we cleared the interaction and global bounds (i.e) reset
   * back to the config or dataset bounds
   */
  #getXResetBounds(): Partial<Bounds1D> {
    const currentSecondsIfFollowMode =
      this.#isTimeseriesPlot && this.#followRange != undefined && this.#currentSeconds != undefined
        ? this.#currentSeconds
        : undefined;
    const xMax = currentSecondsIfFollowMode ?? this.#configBounds.x.max ?? this.#datasetRange?.max;

    const xMinIfFollowMode =
      this.#isTimeseriesPlot && this.#followRange != undefined && xMax != undefined
        ? xMax - this.#followRange
        : undefined;
    const xMin = xMinIfFollowMode ?? this.#configBounds.x.min ?? this.#datasetRange?.min;

    return { min: xMin, max: xMax };
  }

  #getXBounds(): Partial<Bounds1D> {
    // Interaction, synced global bounds override the config and data source bounds in precedence
    const resetBounds = this.#getXResetBounds();
    return {
      min: this.#interactionBounds?.x.min ?? this.#globalBounds?.min ?? resetBounds.min,
      max: this.#interactionBounds?.x.max ?? this.#globalBounds?.max ?? resetBounds.max,
    };
  }

  #isDestroyed(): boolean {
    return this.#destroyed;
  }

  async #dispatchRender(): Promise<void> {
    if (this.#isDestroyed()) {
      return;
    }
    this.#updateAction.xBounds = this.#getXBounds();

    if (this.#shouldResetY) {
      const yMin = this.#interactionBounds?.y.min ?? this.#configBounds.y.min;
      const yMax = this.#interactionBounds?.y.max ?? this.#configBounds.y.max;
      this.#updateAction.yBounds = { min: yMin, max: yMax };
      this.#shouldResetY = false;
    }

    const haveInteractionEvents = (this.#updateAction.interactionEvents?.length ?? 0) > 0;

    const action = this.#updateAction;
    this.#updateAction = {
      type: "update",
    };

    const bounds = await this.#renderer.update(action);
    if (this.#isDestroyed()) {
      return;
    }

    if (haveInteractionEvents) {
      this.#interactionBounds = bounds;
    }

    if (haveInteractionEvents && bounds) {
      this.emit("timeseriesBounds", bounds.x);
    }
    this.emit("viewportChange", this.#canReset());
    this.#queueDispatchDatasets();
  }

  async #dispatchDatasets(): Promise<void> {
    if (this.#isDestroyed()) {
      return;
    }
    this.#viewport.bounds.x = this.#getXBounds();
    this.#viewport.bounds.y = this.#interactionBounds?.y ?? this.#configBounds.y;

    const result = await this.#datasetsBuilder.getViewportDatasets(this.#viewport);
    if (this.#isDestroyed()) {
      return;
    }
    this.#latestXScale = await this.#renderer.updateDatasets(
      // Use Array.from to fill in any `undefined` entries with an empty dataset (`map` would not
      // work for sparse arrays)
      Array.from(result.datasetsByConfigIndex, replaceUndefinedWithEmptyDataset),
    );
    if (this.#isDestroyed()) {
      return;
    }
    this.emit("xScaleChanged", this.#latestXScale);
    this.emit("pathsWithMismatchedDataLengthsChanged", [...result.pathsWithMismatchedDataLengths]);
  }

  async #dispatchBlocks(
    startTime: Immutable<Time>,
    blocks: Immutable<(MessageBlock | undefined)[]>,
  ): Promise<void> {
    if (!this.#datasetsBuilder.handleBlocks) {
      return;
    }

    await this.#datasetsBuilder.handleBlocks(startTime, blocks, async () => {
      this.#queueDispatchDatasets();
      // When blocks are fully loaded and a user splits the panel, we are able to process all of the
      // blocks synchronously. However this creates a poor UX experience for large datasets by
      // showing nothing on the plot for many seconds while the postMessage prepares a massive send.
      // This send also hangs the main thread.
      //
      // Instead of doing this all synchronously and stalling main thread, we artificially break up
      // block loading to periodically dispatch the loaded data and render it. This avoids stalling
      // the main thread and provides visual feedabck to the user that data is loading on the plot.
      //
      // await Promise.resolve() does not work as it does not yield enough to the main thread to
      // dispatch and render.
      await delay(0);

      // Bail processing if the coordinator has been destroyed or if our input blocks have changed
      // This lets us start processing new input blocks instead of continuing to work on stale
      // blocks.
      return this.#isDestroyed() || this.#latestBlocks !== blocks;
    });
  }
}
