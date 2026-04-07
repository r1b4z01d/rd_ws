"""Rendering helpers for the animated eye canvas — FluxGarage RoboEyes style."""

from __future__ import annotations

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Palette (BGR)
# ---------------------------------------------------------------------------
_EYE_BASE_COLOR = (120,  60,  10)   # dark blue fill
_EYE_LINE_COLOR = (255, 185,  30)   # bright electric-blue scanlines
_GLOW_COLOR     = (255, 220, 100)   # top-edge rim / bloom hue

# ---------------------------------------------------------------------------
# Scanline / bloom tuning
# ---------------------------------------------------------------------------
_WAVE_SPACING  = 10    # pixels between scanline rows
_WAVE_AMP      = 2.5   # sine amplitude in pixels
_WAVE_FREQ     = 0.05  # spatial frequency (rad/px)
_GLOW_KERNEL   = 61    # blur kernel size (must be odd); controls bloom radius
_GLOW_STRENGTH = 1.1   # bloom brightness multiplier


def _filled_rounded_rect(canvas, x0, y0, x1, y1, r, color):
    """Fill a rounded rectangle using rects + corner circles."""
    if x1 <= x0 or y1 <= y0 or r < 0:
        return
    r = min(r, (x1 - x0) // 2, (y1 - y0) // 2)
    cv2.rectangle(canvas, (x0 + r, y0), (x1 - r, y1), color, -1)
    cv2.rectangle(canvas, (x0, y0 + r), (x0 + r, y1 - r), color, -1)
    cv2.rectangle(canvas, (x1 - r, y0 + r), (x1, y1 - r), color, -1)
    cv2.circle(canvas, (x0 + r, y0 + r), r, color, -1, cv2.LINE_AA)
    cv2.circle(canvas, (x1 - r, y0 + r), r, color, -1, cv2.LINE_AA)
    cv2.circle(canvas, (x0 + r, y1 - r), r, color, -1, cv2.LINE_AA)
    cv2.circle(canvas, (x1 - r, y1 - r), r, color, -1, cv2.LINE_AA)


def draw_eye(
    canvas,
    center,
    sclera_radius,
    pupil_radius,
    pupil_offset,
    blink_amount,
    eyelid_color,
    draw_eyelashes,
    eyelash_color,
    eyelash_count,
    eyelash_length_top,
    eyelash_length_side,
    eyelash_thickness,
):
    cx, cy = int(center[0]), int(center[1])

    # -----------------------------------------------------------------------
    # Geometry
    # -----------------------------------------------------------------------
    eye_w      = sclera_radius * 2
    eye_h_base = pupil_radius * 2
    eye_h      = int(eye_h_base * max(0.0, 1.0 - float(blink_amount)))

    if eye_h <= 2:
        return

    corner_r = min(eye_w, eye_h) // 4

    dx = float(pupil_offset[0])
    dy = float(pupil_offset[1])
    draw_cx = int(cx + dx)
    draw_cy = int(cy + dy)

    x0 = draw_cx - eye_w // 2
    y0 = draw_cy - eye_h // 2
    x1 = draw_cx + eye_w // 2
    y1 = draw_cy + eye_h // 2

    h, w = canvas.shape[:2]

    # -----------------------------------------------------------------------
    # Work on an isolated layer for bloom compositing
    # -----------------------------------------------------------------------
    eye_layer = np.zeros_like(canvas)

    # Step 1 — dark base fill
    _filled_rounded_rect(eye_layer, x0, y0, x1, y1, corner_r, _EYE_BASE_COLOR)

    # Step 2 — wavy horizontal scanlines
    xs = np.arange(max(0, x0), min(w, x1), dtype=np.float32)
    if len(xs) > 1:
        for row_y in range(y0 + 2, y1 - 2, _WAVE_SPACING):
            ys = row_y + _WAVE_AMP * np.sin(_WAVE_FREQ * xs)
            ys = np.clip(ys, 0, h - 1).astype(np.int32)
            pts = np.column_stack([xs.astype(np.int32), ys]).reshape(-1, 1, 2)
            cv2.polylines(eye_layer, [pts], False, _EYE_LINE_COLOR, 1, cv2.LINE_AA)

    # Step 3 — mood corner cuts (near-black overlay hides unwanted regions)
    mood  = int(eyelash_count)
    cut_w = int(eye_w * 0.35)
    cut_h = int(eye_h * 0.60)
    cc    = eyelid_color

    if mood == 1:   # TIRED — top-outer corners, shallow droop
        cw = int(eye_w * 0.40)
        ch = int(eye_h * 0.50)
        pts = np.array([[x0, y0], [x0 + cw, y0], [x0, y0 + ch]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)
        pts = np.array([[x1, y0], [x1 - cw, y0], [x1, y0 + ch]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)

    elif mood == 2:  # ANGRY — top-inner corners, steep brow
        pts = np.array([[x0, y0], [x0 + cut_w, y0], [x0, y0 + cut_h]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)
        pts = np.array([[x1, y0], [x1 - cut_w, y0], [x1, y0 + cut_h]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)

    elif mood == 3:  # HAPPY — bottom corners squint
        bw = int(eye_w * 0.35)
        bh = int(eye_h * 0.55)
        pts = np.array([[x0, y1], [x0 + bw, y1], [x0, y1 - bh]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)
        pts = np.array([[x1, y1], [x1 - bw, y1], [x1, y1 - bh]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)

    elif mood == 4:  # SUSPICIOUS — top-outer shallow + bottom-inner slight
        cw = int(eye_w * 0.30)
        ch = int(eye_h * 0.45)
        pts = np.array([[x0, y0], [x0 + cw, y0], [x0, y0 + ch]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)
        pts = np.array([[x1, y0], [x1 - cw, y0], [x1, y0 + ch]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)
        bi_w = int(eye_w * 0.20)
        bi_h = int(eye_h * 0.30)
        pts = np.array([[x0, y1], [x0 + bi_w, y1], [x0, y1 - bi_h]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)
        pts = np.array([[x1, y1], [x1 - bi_w, y1], [x1, y1 - bi_h]], dtype=np.int32)
        cv2.fillPoly(eye_layer, [pts], cc, cv2.LINE_AA)

    # Step 4 — bright rim at top edge
    if draw_eyelashes and eye_h > 10:
        cv2.line(
            eye_layer,
            (x0 + corner_r, y0),
            (x1 - corner_r, y0),
            _GLOW_COLOR,
            eyelash_thickness,
            cv2.LINE_AA,
        )

    # -----------------------------------------------------------------------
    # Step 5 — bloom: blur the eye layer and add diffuse glow to canvas,
    #          then composite the sharp layer on top
    # -----------------------------------------------------------------------
    glow = cv2.GaussianBlur(eye_layer, (_GLOW_KERNEL, _GLOW_KERNEL), 0)
    glow_scaled = np.clip(glow.astype(np.float32) * _GLOW_STRENGTH, 0, 255).astype(np.uint8)
    cv2.add(canvas, glow_scaled, canvas)   # diffuse bloom bleeds beyond the eye edge
    cv2.add(canvas, eye_layer, canvas)     # crisp scanline layer on top


__all__ = ["draw_eye"]
