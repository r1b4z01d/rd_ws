"""Keep the robot's speech on the right speaker.

On RD the speaker hangs off the dock's USB audio adapter (C-Media). PulseAudio restores the saved
default sink at start-up, but that adapter enumerates a few seconds after pulse comes up, so the
`set-default-sink` in ~/.config/pulse/default.pa runs before the sink exists and the default lands
on the Jetson's onboard jack. Nothing fixed that after a reboot once the supervisor units replaced
start_face.sh. The helpers here resolve a sink by a name fragment ("C-Media"), re-assert it as the
default whenever it is present (a watchdog thread in the face node), and pin individual players to
it through PULSE_SINK so even a late switch cannot send a sentence to the wrong output.
get/set_volume and set_mute drive that sink's volume for the manager's header slider.

Same file in robot_voice/audio_sink.py; keep them identical.
"""
from __future__ import annotations

import os
import re
import subprocess
import threading
import time
from typing import Callable, List, Optional

DEFAULT_TOKEN = "@DEFAULT_SINK@"


def _pactl(*args: str) -> Optional[str]:
    try:
        r = subprocess.run(["pactl", *args], capture_output=True, text=True, timeout=5)
    except Exception:  # noqa: BLE001 — no pactl / no server
        return None
    return r.stdout if r.returncode == 0 else None


def list_sinks() -> List[str]:
    out = _pactl("list", "sinks", "short") or ""
    return [line.split("\t")[1] for line in out.splitlines() if "\t" in line]


def default_sink() -> Optional[str]:
    out = _pactl("get-default-sink")
    return out.strip() if out else None


def resolve_sink(pattern: str) -> Optional[str]:
    """Full sink name for ``pattern``: an exact name, else a case-insensitive fragment. None when
    the pattern is empty / the default token / no such sink is present."""
    p = (pattern or "").strip()
    if not p or p == DEFAULT_TOKEN:
        return None
    sinks = list_sinks()
    if p in sinks:
        return p
    hits = [s for s in sinks if p.lower() in s.lower()]
    return hits[0] if hits else None


def ensure_default_sink(pattern: str, log: Optional[Callable[[str], None]] = None) -> Optional[str]:
    """Make the sink matching ``pattern`` the default if it exists and isn't already. Returns the
    resolved name (or None when absent)."""
    name = resolve_sink(pattern)
    if name is None:
        return None
    cur = default_sink()
    if cur != name:
        if _pactl("set-default-sink", name) is not None and log:
            log(f"audio: default sink was {cur or 'unset'}; switched to {name}")
    return name


def player_env(pattern: str, base: Optional[dict] = None) -> dict:
    """Environment for aplay/ffplay/paplay with PULSE_SINK pinned to the resolved sink."""
    env = dict(base if base is not None else os.environ)
    name = resolve_sink(pattern)
    if name:
        env["PULSE_SINK"] = name
    return env


def _sink_arg(pattern: str) -> str:
    """The sink pactl should act on: the resolved name, else the default sink (pactl's own token)."""
    return resolve_sink(pattern) or DEFAULT_TOKEN


def get_volume(pattern: str) -> dict:
    """{"sink", "volume" (0-100, the channels' mean), "muted"} of the speaker, or {"error"} without pulse.

    `pactl get-sink-volume` prints "Volume: front-left: 42000 /  64% / -11.6 dB, front-right: ..."."""
    sink = _sink_arg(pattern)
    out = _pactl("get-sink-volume", sink)
    if out is None:
        return {"sink": sink, "error": "pactl failed (no PulseAudio, or no such sink)"}
    pcts = [int(m) for m in re.findall(r"(\d+)%", out)]
    mute = (_pactl("get-sink-mute", sink) or "").strip().lower()
    return {"sink": sink, "volume": round(sum(pcts) / len(pcts)) if pcts else None,
            "muted": mute.endswith("yes")}


def set_volume(pattern: str, percent: float) -> dict:
    """Set the speaker's volume (0-100 %, clamped: above 100 % pulse amplifies and the small speaker
    clips). Returns the state afterwards, as get_volume."""
    pct = max(0, min(100, int(round(float(percent)))))
    sink = _sink_arg(pattern)
    if _pactl("set-sink-volume", sink, f"{pct}%") is None:
        return {"sink": sink, "error": "pactl set-sink-volume failed"}
    return get_volume(pattern)


def set_mute(pattern: str, muted: bool) -> dict:
    sink = _sink_arg(pattern)
    if _pactl("set-sink-mute", sink, "1" if muted else "0") is None:
        return {"sink": sink, "error": "pactl set-sink-mute failed"}
    return get_volume(pattern)


class SinkWatchdog(threading.Thread):
    """Every ``period`` seconds re-assert the wanted sink as the default (covers boot order, the
    dock re-enumerating, and anything else that moves the default). ``current`` holds the resolved
    name for players to pin to."""

    def __init__(self, pattern: str, period: float = 5.0, log: Optional[Callable[[str], None]] = None):
        super().__init__(daemon=True, name="audio-sink-watchdog")
        self.pattern, self.period, self._log = pattern, max(1.0, float(period)), log or (lambda m: None)
        self.current: Optional[str] = None
        self._missing_logged = False
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                name = ensure_default_sink(self.pattern, self._log)
            except Exception as e:  # noqa: BLE001
                name = None
                self._log(f"audio: sink check failed: {e}")
            if name is None and self.pattern and self.pattern != DEFAULT_TOKEN:
                if not self._missing_logged:
                    self._log(f"audio: no sink matching '{self.pattern}' is present "
                              f"(have: {', '.join(list_sinks()) or 'none'}); speech will use the default "
                              f"{default_sink() or '?'} until it appears")
                    self._missing_logged = True
            elif name is not None and self._missing_logged:
                self._log(f"audio: sink {name} is back")
                self._missing_logged = False
            self.current = name
            self._stop.wait(self.period)
