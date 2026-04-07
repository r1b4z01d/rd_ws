"""Standalone visual test for gl_eye_renderer.py — no ROS required.

Run from the repo root:
    python3 src/robot_face/test_eye_renderer.py

Controls:
    SPACE       cycle through expression presets
    Q / ESC     quit
    WASD / arrows   shift gaze
    B           toggle blink
    0-4         set mood directly
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

import glfw
from robot_face.gl_eye_renderer import GlEyeRenderer

# ---------------------------------------------------------------------------
# Canvas / eye config — mirrors combined_face_hand_node.py defaults
# ---------------------------------------------------------------------------
CANVAS_W = 1080
CANVAS_H = 500
EYE_HW   = 187   # half-width  (220 × 0.85)
EYE_HH   = 77    # half-height ( 90 × 0.85)

# Eye centre vertical: halfway down the canvas
EYE_TOP_MARGIN = CANVAS_H // 2 - EYE_HH   # → eye centred at CANVAS_H/2
EYE_SIDE_MARGIN = 80

# ---------------------------------------------------------------------------
# Presets: (label, blink, gaze_offset, mood)
#   mood: 0=neutral 1=tired 2=angry 3=happy 4=suspicious
# ---------------------------------------------------------------------------
PRESETS = [
    ("neutral",        0.0,  (0,    0),  0),
    ("angry",          0.0,  (0,    0),  2),
    ("tired",          0.0,  (0,    0),  1),
    ("happy",          0.0,  (0,    0),  3),
    ("suspicious",     0.0,  (0,    0),  4),
    ("blink 40%",      0.4,  (0,    0),  0),
    ("blink 80%",      0.8,  (0,    0),  0),
    ("look right",     0.0,  (60,   0),  0),
    ("look left",      0.0,  (-60,  0),  0),
    ("look up",        0.0,  (0,  -25),  0),
    ("look down",      0.0,  (0,   25),  0),
    ("angry + squint", 0.35, (35,   0),  2),
    ("tired + down",   0.2,  (0,   18),  1),
]

MOOD_NAMES = {0: "neutral", 1: "tired", 2: "angry", 3: "happy", 4: "suspicious"}

HELP = "SPACE=next  WASD/arrows=gaze  B=blink  0-4=mood  Q=quit"


def main():
    renderer = GlEyeRenderer(
        width=CANVAS_W,
        height=CANVAS_H,
        title=f"Eye Renderer Test — {HELP}",
        eye_hw=float(EYE_HW),
        eye_hh=float(EYE_HH),
        eye_side_margin=float(EYE_SIDE_MARGIN),
        eye_top_margin=float(EYE_TOP_MARGIN),
    )
    win = renderer._window

    preset_idx = 0
    blink      = 0.0
    gaze       = [0.0, 0.0]
    mood       = 0
    manual     = False

    # Edge-detect helpers (only trigger on key-down, not hold)
    prev = {k: False for k in (glfw.KEY_SPACE, glfw.KEY_B)}

    last_t = time.monotonic()

    while not renderer.should_close():
        now = time.monotonic()
        dt  = max(now - last_t, 1e-4)
        last_t = now

        # --- quit ---
        if glfw.get_key(win, glfw.KEY_Q)      == glfw.PRESS:
            break
        if glfw.get_key(win, glfw.KEY_ESCAPE) == glfw.PRESS:
            break

        # --- SPACE: advance preset ---
        space = glfw.get_key(win, glfw.KEY_SPACE) == glfw.PRESS
        if space and not prev[glfw.KEY_SPACE]:
            preset_idx = (preset_idx + 1) % len(PRESETS)
            manual = False
        prev[glfw.KEY_SPACE] = space

        # --- B: toggle blink ---
        b = glfw.get_key(win, glfw.KEY_B) == glfw.PRESS
        if b and not prev[glfw.KEY_B]:
            blink  = 0.0 if blink > 0.0 else 0.75
            manual = True
        prev[glfw.KEY_B] = b

        # --- 0-4: mood ---
        for idx, key in enumerate([
            glfw.KEY_0, glfw.KEY_1, glfw.KEY_2, glfw.KEY_3, glfw.KEY_4
        ]):
            if glfw.get_key(win, key) == glfw.PRESS:
                mood   = idx
                manual = True

        # --- WASD / arrows: gaze ---
        speed = 120.0  # px/s
        if glfw.get_key(win, glfw.KEY_A)     == glfw.PRESS or \
           glfw.get_key(win, glfw.KEY_LEFT)  == glfw.PRESS:
            gaze[0] -= speed * dt;  manual = True
        if glfw.get_key(win, glfw.KEY_D)     == glfw.PRESS or \
           glfw.get_key(win, glfw.KEY_RIGHT) == glfw.PRESS:
            gaze[0] += speed * dt;  manual = True
        if glfw.get_key(win, glfw.KEY_W)     == glfw.PRESS or \
           glfw.get_key(win, glfw.KEY_UP)    == glfw.PRESS:
            gaze[1] -= speed * dt;  manual = True
        if glfw.get_key(win, glfw.KEY_S)     == glfw.PRESS or \
           glfw.get_key(win, glfw.KEY_DOWN)  == glfw.PRESS:
            gaze[1] += speed * dt;  manual = True

        # --- apply preset if not in manual mode ---
        if not manual:
            name, blink, gaze_t, mood = PRESETS[preset_idx]
            gaze = list(gaze_t)
            title = (f"Eye Test  [{preset_idx + 1}/{len(PRESETS)}] {name}"
                     f"  —  {HELP}")
        else:
            title = (f"Eye Test  manual  mood={MOOD_NAMES.get(mood, mood)}"
                     f"  blink={blink:.2f}  gaze=({gaze[0]:.0f},{gaze[1]:.0f})"
                     f"  —  {HELP}")

        glfw.set_window_title(win, title)
        renderer.render(blink=blink, gaze_offset=(gaze[0], gaze[1]), mood=mood, dt=dt)

    renderer.destroy()


if __name__ == "__main__":
    main()
