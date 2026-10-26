"""Mouth gesture library — shapes and timings for the idle mouth animations.

Each gesture is a short performance: ``gesture_frame(name, t, now)`` returns
what the mouth should do at normalised time ``t`` in [0, 1]:

    level   0..1   procedural waveform amplitude (talking-like motion)
    smile   px     extra upward curl of the mouth ends (negative = frown)
    skew    px     tilt (right end up / left end down)
    points  (N,2)  optional polyline shape, or None
    blend   0..1   how far to morph from the resting curve into ``points``

Shape coordinates are normalised: x in units of the mouth half-width (-1..1),
y in units of the waveform amplitude (positive = down, matching screen Y).
The renderer scales them to pixels.

No ROS dependency: the standalone test harness uses this module too.
"""

from __future__ import annotations

import math
import random
from typing import Callable, Dict, Optional, Tuple

import numpy as np

ShapeFn = Callable[[float, float, int], np.ndarray]   # (t, now_seconds, side) -> (N, 2)


def _ease(t: float) -> float:
    """Smooth in/out envelope over t in [0, 1]."""
    return math.sin(math.pi * min(max(t, 0.0), 1.0))


def _hold_env(t: float, attack: float = 0.25) -> float:
    """Ease in over ``attack``, hold at 1, ease out over the last ``attack``."""
    if t < attack:
        return 0.5 - 0.5 * math.cos(math.pi * t / attack)
    if t > 1.0 - attack:
        return 0.5 - 0.5 * math.cos(math.pi * (1.0 - t) / attack)
    return 1.0


def _poly(*pts) -> np.ndarray:
    return np.asarray(pts, dtype=np.float32)


def _curve(fn, x0: float, x1: float, n: int = 64) -> np.ndarray:
    xs = np.linspace(x0, x1, n, dtype=np.float32)
    ys = np.asarray([fn(float(x)) for x in xs], dtype=np.float32)
    return np.column_stack([xs, ys])


# ---------------------------------------------------------------------------
# Shape generators
# ---------------------------------------------------------------------------
def skeptical(t, now, side):
    """Straight diagonal line slanted down to one side."""
    return _poly((-0.75, -0.35 * side), (0.75, 0.45 * side))


def shocked(t, now, side):
    """Small closed oval — an 'O'."""
    # Start at the left so the morph from the resting line doesn't twist
    th = np.linspace(math.pi, 3.0 * math.pi, 48, dtype=np.float32)
    rx, ry = 0.17, 1.0
    pts = np.column_stack([rx * np.cos(th), 0.15 + ry * np.sin(th)]).astype(np.float32)
    pts[-1] = pts[0]                                      # close the loop exactly
    return pts


def puckered(t, now, side):
    """Sideways '3' / 'w' drawn as one stroke."""
    w, a = 0.24, 0.55
    return _curve(lambda x: -a * math.cos(2.0 * math.pi * x / w) * 0.5 + 0.1, -w, w, 48)


def anxious(t, now, side):
    """Sharp jagged zig-zag with a nervous shake."""
    n = 11
    xs = np.linspace(-0.72, 0.72, n, dtype=np.float32)
    ys = np.where(np.arange(n) % 2 == 0, -0.45, 0.45).astype(np.float32)
    ys += 0.05 * math.sin(now * 37.0) * np.where(np.arange(n) % 2 == 0, 1.0, -1.0)
    return np.column_stack([xs, ys])


def slack_jawed(t, now, side):
    """Deep elongated 'U' drooping at the bottom."""
    w, depth = 0.45, 1.9
    return _curve(lambda x: -0.25 + depth * (1.0 - (x / w) ** 2), -w, w, 48)


def playful(t, now, side):
    """Flat line with a small downward loop in the middle — a tongue."""
    r, ry, w = 0.11, 1.5, 0.72
    wag = 0.25 * math.sin(now * 9.0)                       # tongue wags a little
    th = np.linspace(0.0, math.pi, 28, dtype=np.float32)
    loop = np.column_stack([-r * np.cos(th) + wag * r * np.sin(th), ry * np.sin(th)])
    return np.vstack([
        _poly((-w, 0.0), (-r, 0.0)),
        loop.astype(np.float32),
        _poly((r, 0.0), (w, 0.0)),
    ]).astype(np.float32)


def grimace(t, now, side):
    """Sharp upside-down 'V' (^)."""
    return _poly((-0.6, 0.6), (0.0, -0.6), (0.6, 0.6))


def shouting(t, now, side):
    """Large, sharp right-side-up 'V'."""
    return _poly((-0.75, -1.1), (0.0, 1.2), (0.75, -1.1))


def quivering(t, now, side):
    """Gentle curve with tiny shaky stutters along it."""
    curve = 0.35 * side                                   # side>0: frown-ish, side<0: smile-ish
    def fn(x):
        base = curve * (x / 0.7) ** 2 - curve * 0.5
        return base + 0.045 * math.sin(x * 70.0 + now * 55.0) + 0.03 * math.sin(now * 23.0)
    return _curve(fn, -0.7, 0.7, 72)


def puzzled(t, now, side):
    """Asymmetric check mark: slightly down, then sharply up to one side."""
    pts = _poly((-0.7, -0.05), (0.05, 0.5), (0.6, -1.3))
    pts[:, 0] *= side
    return pts


# ---------------------------------------------------------------------------
# Gesture table
#   duration (s), level, smile px, skew px, shape fn (or None), random side?
# ---------------------------------------------------------------------------
GESTURES: Dict[str, dict] = {
    # motion-only gestures (no shape change)
    "hum":         dict(duration=0.9, level=0.35, smile=0.0,  skew=0.0,   shape=None,        side=False),
    "grin":        dict(duration=1.8, level=0.0,  smile=45.0, skew=0.0,   shape=None,        side=False),
    "pout":        dict(duration=1.4, level=0.0,  smile=-45.0, skew=0.0,  shape=None,        side=False),
    "smirk":       dict(duration=1.6, level=0.0,  smile=10.0, skew=28.0,  shape=None,        side=True),
    "wiggle":      dict(duration=1.2, level=0.12, smile=0.0,  skew=18.0,  shape=None,        side=False, rock=True),
    # shape gestures
    "skeptical":   dict(duration=2.2, level=0.0,  smile=0.0,  skew=0.0,   shape=skeptical,   side=True),
    "shocked":     dict(duration=1.8, level=0.0,  smile=0.0,  skew=0.0,   shape=shocked,     side=False),
    "puckered":    dict(duration=1.6, level=0.0,  smile=0.0,  skew=0.0,   shape=puckered,    side=False),
    "anxious":     dict(duration=1.8, level=0.0,  smile=0.0,  skew=0.0,   shape=anxious,     side=False),
    "slack_jawed": dict(duration=2.4, level=0.0,  smile=0.0,  skew=0.0,   shape=slack_jawed, side=False),
    "playful":     dict(duration=2.0, level=0.0,  smile=0.0,  skew=0.0,   shape=playful,     side=False),
    "grimace":     dict(duration=1.6, level=0.0,  smile=0.0,  skew=0.0,   shape=grimace,     side=False),
    "shouting":    dict(duration=1.2, level=0.5,  smile=0.0,  skew=0.0,   shape=shouting,    side=False),
    "quivering":   dict(duration=2.2, level=0.0,  smile=0.0,  skew=0.0,   shape=quivering,   side=True),
    "puzzled":     dict(duration=2.2, level=0.0,  smile=0.0,  skew=0.0,   shape=puzzled,     side=True),
}

SHAPE_GESTURES = [k for k, v in GESTURES.items() if v["shape"] is not None]


def pick_gesture(rng: random.Random = random) -> Tuple[str, int]:
    """Choose a random gesture name and a side (+1 / -1)."""
    name = rng.choice(list(GESTURES))
    side = rng.choice((-1, 1)) if GESTURES[name]["side"] else 1
    return name, side


def gesture_duration(name: str) -> float:
    return float(GESTURES[name]["duration"])


def gesture_frame(
    name: str, t: float, now: float, side: int = 1
) -> Tuple[float, float, float, Optional[np.ndarray], float]:
    """Evaluate gesture ``name`` at normalised time ``t`` (0..1).

    Returns (level, smile_px, skew_px, points_or_None, blend).
    """
    g = GESTURES[name]
    if g["shape"] is None:
        env = _ease(t)
        skew = g["skew"] * side
        if g.get("rock"):
            skew *= math.sin(now * 14.0)
        return g["level"] * env, g["smile"] * env, skew * env, None, 0.0

    env = _hold_env(t)                                    # morph in, hold the shape, morph out
    pts = g["shape"](t, now, side)
    return g["level"] * env, 0.0, 0.0, pts, env


__all__ = ["GESTURES", "SHAPE_GESTURES", "pick_gesture", "gesture_duration", "gesture_frame"]
