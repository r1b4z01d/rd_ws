"""Standalone visual test for gl_eye_renderer.py — no ROS required.

Run from the repo root:
    python3 src/robot_face/test_eye_renderer.py

Controls:
    SPACE       cycle through expression presets
    Q / ESC     quit
    WASD / arrows   shift gaze
    B           toggle blink
    0-4         set mood directly
    M           toggle simulated talking (procedural mouth waveform)
    N           toggle simulated talking with synthesised audio samples
    G           cycle through every mouth gesture (press again to stop)
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

import glfw
import numpy as np
from robot_face.gl_eye_renderer import GlEyeRenderer
from robot_face.mouth_gestures import GESTURES, gesture_duration, gesture_frame

# ---------------------------------------------------------------------------
# Canvas / eye config — mirrors combined_face_hand_node.py defaults
# ---------------------------------------------------------------------------
CANVAS_W = 1080
CANVAS_H = 600
EYE_HW   = 187   # half-width  (220 × 0.85)
EYE_HH   = 77    # half-height ( 90 × 0.85)

# Eye centre vertical: upper part of the canvas, leaving room for the mouth
EYE_TOP_MARGIN = 160
EYE_SIDE_MARGIN = 80

# Mouth waveform — eye bottom edge → waveform centre, and peak displacement
MOUTH_GAP = 150
MOUTH_AMP = 70
MOUTH_SMILE = 25         # resting curl of the mouth ends (px)
MOUTH_SMILE_HAPPY = 70   # curl used for the happy mood

# ---------------------------------------------------------------------------
# Presets: (label, blink, gaze_offset, mood, mouth_level)
#   mood: 0=neutral 1=tired 2=angry 3=happy 4=suspicious
# ---------------------------------------------------------------------------
PRESETS = [
    ("neutral",        0.0,  (0,    0),  0, 0.0),
    ("talking",        0.0,  (0,    0),  0, 0.8),
    ("whisper",        0.0,  (0,    0),  0, 0.25),
    ("angry",          0.0,  (0,    0),  2, 0.0),
    ("tired",          0.0,  (0,    0),  1, 0.0),
    ("happy",          0.0,  (0,    0),  3, 0.0),
    ("happy + talking", 0.0, (0,    0),  3, 0.7),
    ("suspicious",     0.0,  (0,    0),  4, 0.0),
    ("blink 40%",      0.4,  (0,    0),  0, 0.0),
    ("blink 80%",      0.8,  (0,    0),  0, 0.0),
    ("look right",     0.0,  (60,   0),  0, 0.0),
    ("look left",      0.0,  (-60,  0),  0, 0.0),
    ("look up",        0.0,  (0,  -25),  0, 0.0),
    ("look down",      0.0,  (0,   25),  0, 0.0),
    ("angry + squint", 0.35, (35,   0),  2, 0.0),
    ("tired + down",   0.2,  (0,   18),  1, 0.0),
]

MOOD_NAMES = {0: "neutral", 1: "tired", 2: "angry", 3: "happy", 4: "suspicious"}

HELP = "SPACE=next  WASD/arrows=gaze  B=blink  0-4=mood  M/N=talk  G=gestures  Q=quit"
GESTURE_NAMES = list(GESTURES)

SAMPLE_COUNT = 512


def talk_envelope(t: float) -> float:
    """Syllable-like loudness envelope in [0, 1]."""
    syllables = abs(math.sin(t * 6.0))
    phrase    = 0.55 + 0.45 * math.sin(t * 1.7 + 1.0)
    return max(0.0, min(1.0, syllables * phrase))


def talk_samples(t: float, level: float) -> np.ndarray:
    """Synthesised 'audio' chunk in [-1, 1] for the sampled waveform mode."""
    xs = np.linspace(0.0, 1.0, SAMPLE_COUNT, dtype=np.float32)
    wave = (
        0.70 * np.sin(xs * 55.0 + t * 40.0)
        + 0.25 * np.sin(xs * 170.0 - t * 95.0)
        + 0.10 * np.sin(xs * 420.0 + t * 210.0)
    )
    return (wave * level).astype(np.float32)


def main():
    renderer = GlEyeRenderer(
        width=CANVAS_W,
        height=CANVAS_H,
        title=f"Eye Renderer Test — {HELP}",
        eye_hw=float(EYE_HW),
        eye_hh=float(EYE_HH),
        eye_side_margin=float(EYE_SIDE_MARGIN),
        eye_top_margin=float(EYE_TOP_MARGIN),
        mouth_gap=float(MOUTH_GAP),
        mouth_amp=float(MOUTH_AMP),
        mouth_samples=SAMPLE_COUNT,
    )
    win = renderer._window

    preset_idx = 0
    blink      = 0.0
    gaze       = [0.0, 0.0]
    mood       = 0
    mouth      = 0.0
    manual     = False
    talk_mode  = 0   # 0 = off, 1 = procedural level, 2 = synthesised samples
    gesture_on    = False
    gesture_idx   = 0
    gesture_start = 0.0
    gesture_side  = 1

    # Edge-detect helpers (only trigger on key-down, not hold)
    prev = {k: False for k in (glfw.KEY_SPACE, glfw.KEY_B, glfw.KEY_M, glfw.KEY_N, glfw.KEY_G)}

    t0     = time.monotonic()
    last_t = t0

    while not renderer.should_close():
        now = time.monotonic()
        dt  = max(now - last_t, 1e-4)
        last_t = now
        t = now - t0

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
            talk_mode = 0
        prev[glfw.KEY_SPACE] = space

        # --- B: toggle blink ---
        b = glfw.get_key(win, glfw.KEY_B) == glfw.PRESS
        if b and not prev[glfw.KEY_B]:
            blink  = 0.0 if blink > 0.0 else 0.75
            manual = True
        prev[glfw.KEY_B] = b

        # --- M / N: toggle simulated talking ---
        m = glfw.get_key(win, glfw.KEY_M) == glfw.PRESS
        if m and not prev[glfw.KEY_M]:
            talk_mode = 0 if talk_mode == 1 else 1
            manual = True
        prev[glfw.KEY_M] = m

        n = glfw.get_key(win, glfw.KEY_N) == glfw.PRESS
        if n and not prev[glfw.KEY_N]:
            talk_mode = 0 if talk_mode == 2 else 2
            manual = True
        prev[glfw.KEY_N] = n

        # --- G: cycle through all mouth gestures ---
        g = glfw.get_key(win, glfw.KEY_G) == glfw.PRESS
        if g and not prev[glfw.KEY_G]:
            gesture_on = not gesture_on
            gesture_idx, gesture_start, gesture_side = 0, now, 1
            manual = True
        prev[glfw.KEY_G] = g

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

        # --- mouth: preset level, or simulated talking ---
        samples = None
        g_smile, g_skew, g_points, g_blend, g_level, g_name = 0.0, 0.0, None, 0.0, 0.0, ""
        if gesture_on:
            g_name = GESTURE_NAMES[gesture_idx]
            tg = (now - gesture_start) / gesture_duration(g_name)
            if tg >= 1.0:                                   # next gesture (with a short pause)
                if now - gesture_start > gesture_duration(g_name) + 0.6:
                    gesture_idx = (gesture_idx + 1) % len(GESTURE_NAMES)
                    gesture_start, gesture_side = now, -gesture_side
            else:
                g_level, g_smile, g_skew, g_points, g_blend = gesture_frame(
                    g_name, tg, now, gesture_side
                )
        if not manual:
            name, blink, gaze_t, mood, mouth = PRESETS[preset_idx]
            gaze = list(gaze_t)
            title = (f"Eye Test  [{preset_idx + 1}/{len(PRESETS)}] {name}"
                     f"  —  {HELP}")
        else:
            if talk_mode == 1:
                mouth = talk_envelope(t)
            elif talk_mode == 2:
                mouth = talk_envelope(t)
                samples = talk_samples(t, mouth)
            else:
                mouth = 0.0
            talk_str = {0: "off", 1: "procedural", 2: "sampled"}[talk_mode]
            gest_str = f"  gesture={g_name}" if gesture_on else ""
            title = (f"Eye Test  manual  mood={MOOD_NAMES.get(mood, mood)}"
                     f"  blink={blink:.2f}  gaze=({gaze[0]:.0f},{gaze[1]:.0f})"
                     f"  talk={talk_str}{gest_str}  —  {HELP}")

        glfw.set_window_title(win, title)
        renderer.render(
            blink=blink,
            gaze_offset=(gaze[0], gaze[1]),
            mood=mood,
            dt=dt,
            mouth_level=max(mouth, g_level),
            mouth_samples=samples,
            mouth_smile=(MOUTH_SMILE_HAPPY if mood == 3 else MOUTH_SMILE) + g_smile,
            mouth_skew=g_skew,
            mouth_shape=g_blend,
            mouth_path=g_points,
        )

    renderer.destroy()


if __name__ == "__main__":
    main()
