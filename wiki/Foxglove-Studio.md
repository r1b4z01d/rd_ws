# Foxglove Studio

**Path**: `foxglove_studio/`

A full local clone of the open-source Foxglove Studio TypeScript monorepo, customized for Robot Disco, plus the docker image and layout the robot serves. Since 2026-11-01 both the bridge and the studio are **manual-start** units (`sudo supervisorctl start robot_disco_foxglove_bridge robot_disco_foxglove_studio`, then `http://192.168.11.80:8080/`): the route editor's **3D view** (`http://192.168.11.80:8092/3d`, the manager's Map tab) replaced it for daily use (map, costmaps, lidar and Velodyne clouds, URDF with live joints, TF, Nav2 plans/goal/footprint/polygons/particles, AprilTags, cameras, HALT), with no Docker and no internet needed. Foxglove stays for anything the pages do not show (arbitrary topics, plots, the service-call panel).

## Custom Panel: `CallPartyService`

**Path**: `foxglove_studio/packages/studio-base/src/panels/CallPartyService/`

A bespoke Foxglove panel that calls ROS2 services directly from the browser UI. Fully configurable via the Foxglove settings sidebar:

| Setting | Description |
|---|---|
| Service name | ROS2 service to call (e.g. `/start_waypoints`) |
| Button text | Label shown on the button |
| Tooltip | Hover text |
| Button color | RGBA color picker |
| Request payload | JSON body sent with the service call |
| Layout | Horizontal or vertical button arrangement |

State tracking: button reflects `requesting` / `success` / `error` states.

## Dashboard Layout (`foxglove-layout.json`)

The default layout loaded into Foxglove Studio at startup:

| Panel | Topic / Service | Description |
|---|---|---|
| Battery Gauge | `/battery.percentage` | 0–1 range, red-yellow-green colormap |
| Power Supply Indicator | `/battery.power_supply_status` | Charging state display |
| E-Stop Button | `/stop_motor` (service) | Large red emergency stop button |
| 3D View | `/map`, `/velodyne_points`, `/scan`, `/robot_description`, `/camera/camera/color/image_raw`, `/waypoints_path` | Full sensor + map visualization |
| Image Panel | `/camera/camera/color/image_raw` | Camera feed with pose estimation overlay |
| Party Buttons | `/start_waypoints`, `/Chargers` | "Party In Living Room" (purple), "Go To Charger" (green) |
| ROS Out | — | Log viewer |

3D view layers:
- `/global_costmap/costmap` at 40% alpha
- `/velodyne_points` — rainbow colormap, circle points, 2 px size
- `/scan` — turbo colormap
- Camera image projected into 3D scene with calibration from `/camera/camera/color/camera_info`
- Waypoint path and goal pose topics

How the default is applied: the container entrypoint injects `/foxglove/default-layout.json` (bind-mounted from `foxglove_studio/foxglove-layout.json` by the supervisor conf) into `index.html` as `FOXGLOVE_STUDIO_DEFAULT_LAYOUT`. Studio uses it **only for browsers that have no saved layout** — it persists the current layout in `localStorage` under `studio.layout`. After changing `foxglove-layout.json`, restart `robot_disco_foxglove_studio` and have existing clients clear that key (`localStorage.removeItem('studio.layout')` in the devtools console, then reload) or open the page in a fresh browser profile. To check the injection worked: `curl -s localhost:8080 | grep -o 'FOXGLOVE_STUDIO_DEFAULT_LAYOUT = .\{0,40\}'` on the robot should show `[{"configById"…`, not `[][0]`.

## Docker Deployment (`compose.foxglove.yaml`)

```yaml
services:
  foxglove:
    image: icon/foxglove
    ports:
      - 80:8080
    volumes:
      - ./foxglove-layout.json:/foxglove/default-layout.json
    environment:
      - DS_TYPE=foxglove-websocket
      - DS_PORT=8765
      - UI_PORT=8080
      - DISABLE_CACHE=true
```

Web UI is served on port 8080 (`--publish 8080:8080` in the supervisor conf; `compose.foxglove.yaml` maps it to 80 instead). Connects to the ROS2 Foxglove WebSocket bridge on port 8765.

```bash
# Start the Foxglove web UI
docker run --rm --name foxglove \
  --publish 8080:8080 \
  --volume ./foxglove_studio/foxglove-layout.json:/foxglove/default-layout.json \
  robotdisco/foxglove

# Or via compose
docker compose -f foxglove_studio/compose.foxglove.yaml up
```
