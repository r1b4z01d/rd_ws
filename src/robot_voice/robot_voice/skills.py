"""Assistant skills with no ROS dependency: weather, news, Home Assistant,
local music, and the clock. Each returns a short *spoken* sentence.

Navigation lives in the node (it needs the Nav2 action client) and is passed
to the brain as a ``Navigator`` object with ``go_to(place)``, ``stop()`` and
``places()``.
"""

from __future__ import annotations

import difflib
import json
import os
import random
import re
import subprocess
import shutil
import threading
import time
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from datetime import datetime
from typing import Callable, Dict, List, Optional, Sequence

from .audio_sink import DEFAULT_TOKEN, player_env, resolve_sink

UA = {"User-Agent": "RobotDisco/1.0 (robot voice assistant)"}


def _get_json(url: str, headers: Optional[dict] = None, timeout: float = 8.0):
    req = urllib.request.Request(url, headers={**UA, **(headers or {})})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def _fuzzy_pick(query: str, candidates: Sequence[str], cutoff: float = 0.55) -> Optional[str]:
    """Best fuzzy match of ``query`` among ``candidates`` (case-insensitive)."""
    q = query.lower().strip()
    if not q or not candidates:
        return None
    low = {c.lower(): c for c in candidates}
    if q in low:
        return low[q]
    # substring / word containment first
    contains = [c for c in candidates if q in c.lower() or c.lower() in q]
    if contains:
        return min(contains, key=len)
    hits = difflib.get_close_matches(q, list(low.keys()), n=1, cutoff=cutoff)
    return low[hits[0]] if hits else None


# ---------------------------------------------------------------------------
# Clock
# ---------------------------------------------------------------------------
class ClockSkill:
    def time(self) -> str:
        return datetime.now().strftime("It's %-I:%M %p.")

    def date(self) -> str:
        return datetime.now().strftime("Today is %A, %B %-d.")


# ---------------------------------------------------------------------------
# Weather (Open-Meteo, free, no key)
# ---------------------------------------------------------------------------
_WMO = {
    0: "clear", 1: "mostly clear", 2: "partly cloudy", 3: "overcast", 45: "foggy", 48: "foggy",
    51: "drizzling", 53: "drizzling", 55: "drizzling", 56: "freezing drizzle", 57: "freezing drizzle",
    61: "light rain", 63: "rain", 65: "heavy rain", 66: "freezing rain", 67: "freezing rain",
    71: "light snow", 73: "snow", 75: "heavy snow", 77: "snow grains", 80: "rain showers",
    81: "rain showers", 82: "heavy showers", 85: "snow showers", 86: "snow showers",
    95: "thunderstorms", 96: "thunderstorms with hail", 99: "thunderstorms with hail",
}


class WeatherSkill:
    def __init__(self, latitude: float = 0.0, longitude: float = 0.0, place: str = "",
                 units: str = "fahrenheit"):
        self.lat, self.lon, self.place = float(latitude), float(longitude), place
        self.units = "fahrenheit" if units.lower().startswith("f") else "celsius"
        self._cache = (0.0, None)

    def available(self) -> bool:
        return bool(self.lat or self.lon) or bool(self.place)

    def _resolve(self) -> None:
        if (self.lat or self.lon) or not self.place:
            return
        data = _get_json("https://geocoding-api.open-meteo.com/v1/search?"
                         + urllib.parse.urlencode({"name": self.place, "count": 1, "language": "en"}))
        res = (data.get("results") or [None])[0]
        if not res:
            raise ValueError(f"could not find a place called {self.place}")
        self.lat, self.lon = float(res["latitude"]), float(res["longitude"])
        self.place = res.get("name", self.place)

    def fetch(self) -> dict:
        self._resolve()
        ts, cached = self._cache
        if cached is not None and time.time() - ts < 600:
            return cached
        q = {
            "latitude": self.lat, "longitude": self.lon,
            "current": "temperature_2m,apparent_temperature,precipitation,weather_code,wind_speed_10m",
            "daily": "weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max",
            "temperature_unit": self.units, "wind_speed_unit": "mph" if self.units == "fahrenheit" else "kmh",
            "timezone": "auto", "forecast_days": 3,
        }
        data = _get_json("https://api.open-meteo.com/v1/forecast?" + urllib.parse.urlencode(q))
        self._cache = (time.time(), data)
        return data

    def report(self, day: str = "today") -> str:
        if not self.available():
            return "I don't know where I am yet. Set my weather location in the config."
        d = self.fetch()
        unit = "degrees"
        cur, daily = d.get("current", {}), d.get("daily", {})
        where = f" in {self.place}" if self.place else ""
        day = (day or "today").lower()
        idx = 1 if "tomorrow" in day else 0
        try:
            hi, lo = round(daily["temperature_2m_max"][idx]), round(daily["temperature_2m_min"][idx])
            code = _WMO.get(int(daily["weather_code"][idx]), "mixed")
            pop = daily.get("precipitation_probability_max", [None] * 3)[idx]
        except (KeyError, IndexError, TypeError):
            return "The weather service didn't give me a forecast."
        rain = f", with a {int(pop)} percent chance of rain" if pop not in (None, 0) else ""
        if idx == 0 and cur:
            now = round(cur.get("temperature_2m", 0))
            feels = round(cur.get("apparent_temperature", now))
            feels_txt = f", feels like {feels}" if abs(feels - now) >= 3 else ""
            return (f"Right now it's {now} {unit}{feels_txt} and {_WMO.get(int(cur.get('weather_code', 0)), 'mixed')}{where}. "
                    f"Today's high is {hi} with a low of {lo}{rain}.")
        return f"Tomorrow{where} looks {code}, with a high of {hi} and a low of {lo}{rain}."


# ---------------------------------------------------------------------------
# News (RSS / Atom headlines)
# ---------------------------------------------------------------------------
class NewsSkill:
    def __init__(self, feeds: Optional[Dict[str, str]] = None):
        self.feeds = feeds or {
            "NPR": "https://feeds.npr.org/1001/rss.xml",
            "BBC": "https://feeds.bbci.co.uk/news/world/rss.xml",
        }
        self._cache: Dict[str, tuple] = {}

    def headlines(self, source: str = "", count: int = 4) -> str:
        if not self.feeds:
            return "I don't have any news feeds configured."
        name = _fuzzy_pick(source, list(self.feeds.keys())) if source else None
        name = name or next(iter(self.feeds))
        url = self.feeds[name]
        ts, items = self._cache.get(name, (0.0, None))
        if items is None or time.time() - ts > 900:
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=8) as r:
                root = ET.fromstring(r.read())
            items = []
            for el in root.iter():
                tag = el.tag.split("}")[-1]
                if tag in ("item", "entry"):
                    title = next((c.text for c in el if c.tag.split("}")[-1] == "title" and c.text), None)
                    if title:
                        items.append(re.sub(r"\s+", " ", title).strip())
            self._cache[name] = (time.time(), items)
        if not items:
            return f"I couldn't get any headlines from {name}."
        top = items[: max(1, min(int(count), 8))]
        joined = ". ".join(top)
        return f"Here are the top headlines from {name}. {joined}."


# ---------------------------------------------------------------------------
# Home Assistant (REST API)
# ---------------------------------------------------------------------------
_HA_DOMAINS = ("light", "switch", "fan", "cover", "media_player", "scene", "script", "climate", "lock",
               "input_boolean", "vacuum", "humidifier")


class HomeAssistantSkill:
    def __init__(self, url: str = "", token: str = "", aliases: Optional[Dict[str, str]] = None):
        self.url = (url or os.environ.get("HASS_URL", "")).rstrip("/")
        self.token = token or os.environ.get("HASS_TOKEN", "") or os.environ.get("HOME_ASSISTANT_TOKEN", "")
        self.aliases = {k.lower(): v for k, v in (aliases or {}).items()}
        self._states = (0.0, [])

    def available(self) -> bool:
        return bool(self.url and self.token)

    def _headers(self) -> dict:
        return {"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"}

    def states(self) -> List[dict]:
        ts, st = self._states
        if st and time.time() - ts < 30:
            return st
        st = _get_json(f"{self.url}/api/states", headers=self._headers())
        self._states = (time.time(), st)
        return st

    def names(self, limit: int = 40) -> List[str]:
        """Friendly names of controllable entities (for the brain's context)."""
        try:
            out = []
            for s in self.states():
                if s["entity_id"].split(".")[0] in _HA_DOMAINS:
                    out.append(s.get("attributes", {}).get("friendly_name") or s["entity_id"])
            return sorted(set(out))[:limit]
        except Exception:
            return []

    def resolve(self, target: str) -> Optional[str]:
        t = target.lower().strip()
        if t in self.aliases:
            return self.aliases[t]
        if "." in t and " " not in t:
            return t
        try:
            states = self.states()
        except Exception:
            return None
        cands = {}
        for s in states:
            eid = s["entity_id"]
            if eid.split(".")[0] not in _HA_DOMAINS:
                continue
            fname = s.get("attributes", {}).get("friendly_name") or eid
            cands[fname] = eid
        pick = _fuzzy_pick(t, list(cands.keys()))
        if pick is None:
            # try without a trailing device word ("kitchen light" → "kitchen")
            for suffix in (" light", " lights", " lamp", " switch", " fan"):
                if t.endswith(suffix):
                    pick = _fuzzy_pick(t[: -len(suffix)], list(cands.keys()))
                    if pick:
                        break
        return cands.get(pick) if pick else None

    def call(self, domain: str, service: str, entity_id: str, data: Optional[dict] = None) -> None:
        body = json.dumps({"entity_id": entity_id, **(data or {})}).encode()
        req = urllib.request.Request(f"{self.url}/api/services/{domain}/{service}", data=body,
                                     headers=self._headers(), method="POST")
        with urllib.request.urlopen(req, timeout=8):
            pass
        self._states = (0.0, [])

    def turn(self, target: str, action: str) -> str:
        if not self.available():
            return "Home Assistant isn't connected. Set its URL and token in my config."
        eid = self.resolve(target)
        if not eid:
            return f"I couldn't find anything called {target} in Home Assistant."
        domain = eid.split(".")[0]
        action = action.lower()
        service = {"on": "turn_on", "turn_on": "turn_on", "off": "turn_off", "turn_off": "turn_off",
                   "toggle": "toggle", "open": "open_cover", "close": "close_cover",
                   "lock": "lock", "unlock": "unlock"}.get(action, action)
        if domain == "cover" and service in ("turn_on", "turn_off"):
            service = "open_cover" if service == "turn_on" else "close_cover"
        if domain in ("scene", "script"):
            service = "turn_on"
        try:
            self.call(domain, service, eid)
        except Exception as e:
            return f"Home Assistant refused that: {e}"
        name = eid.split(".", 1)[1].replace("_", " ")
        verb = {"turn_on": "on", "turn_off": "off", "toggle": "toggled", "open_cover": "opening",
                "close_cover": "closing", "lock": "locked", "unlock": "unlocked"}.get(service, service)
        return f"Okay, {name} {verb}." if verb in ("on", "off") else f"Okay, {name} {verb}."

    def state_of(self, target: str) -> str:
        if not self.available():
            return "Home Assistant isn't connected."
        eid = self.resolve(target)
        if not eid:
            return f"I couldn't find anything called {target}."
        for s in self.states():
            if s["entity_id"] == eid:
                name = s.get("attributes", {}).get("friendly_name") or eid
                st = s.get("state")
                unit = s.get("attributes", {}).get("unit_of_measurement", "")
                return f"{name} is {st}{(' ' + unit) if unit else ''}."
        return f"I have no state for {target}."


# ---------------------------------------------------------------------------
# Music (local files through a command-line player)
# ---------------------------------------------------------------------------
_AUDIO_EXT = (".mp3", ".flac", ".wav", ".m4a", ".ogg", ".opus", ".aac", ".aiff", ".aif")


class MusicSkill:
    """Local music through ffplay with a queue: play builds a shuffled queue (the whole library, an
    artist/album folder, or a requested song followed by the rest), auto-advances when a track ends,
    and supports next / previous / now playing. duck() lowers the player's own PulseAudio stream
    while the robot listens and talks so the wake word and the reply are heard over the music."""

    def __init__(self, music_dir: str = "~/Music", player_cmd: Optional[List[str]] = None, sink: str = "",
                 continuous: bool = True, duck_level: float = 0.25):
        self.sink = sink                                   # PulseAudio sink fragment for PULSE_SINK
        self.music_dir = os.path.expanduser(music_dir)
        self.player_cmd = player_cmd or ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet"]
        self.continuous = continuous
        self.duck_level = max(0.0, min(float(duck_level), 1.0))
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.RLock()
        self._gen = 0                                      # bumps on every start/stop; stale watchers bail out
        self.queue: List[str] = []
        self.index = -1
        self.now_playing: Optional[str] = None
        self._ducked = False

    def library(self) -> List[str]:
        out = []
        for root, _dirs, files in os.walk(self.music_dir):
            for f in files:
                if f.lower().endswith(_AUDIO_EXT):
                    out.append(os.path.join(root, f))
        return sorted(out)

    @staticmethod
    def _label(path: str) -> str:
        name = os.path.splitext(os.path.basename(path))[0]
        name = re.sub(r"^\s*\d+[.\s-]+", "", name)          # leading track numbers
        return re.sub(r"\s+", " ", re.sub(r"[_\-]+", " ", name)).strip() or os.path.basename(path)

    # -- queue -------------------------------------------------------------------
    def play(self, query: str = "") -> str:
        lib = self.library()
        if not lib:
            return f"I don't have any music in {self.music_dir}."
        rest = list(lib)
        random.shuffle(rest)
        if query:
            labels = {self._label(p): p for p in lib}
            folders = {os.path.basename(os.path.dirname(p)): p for p in lib}
            pick = _fuzzy_pick(query, list(labels.keys()))
            if pick:
                first = labels[pick]
                queue = [first] + [p for p in rest if p != first]
            else:
                fpick = _fuzzy_pick(query, list(folders.keys()))
                if fpick:
                    folder = os.path.dirname(folders[fpick])
                    queue = [p for p in rest if p.startswith(folder)]
                else:
                    return f"I couldn't find {query} in my music."
        else:
            queue = rest
        with self._lock:
            self.queue, self.index = queue, 0
            return self._start(queue[0], "Playing")

    def next(self, auto: bool = False) -> str:
        with self._lock:
            if not self.queue:
                return "Nothing is queued; say play some music."
            if self.index + 1 >= len(self.queue):
                if not self.continuous and auto:
                    self.stop(quiet=True)
                    return ""
                random.shuffle(self.queue)                 # wrap around with a fresh order
                self.index = 0
            else:
                self.index += 1
            return self._start(self.queue[self.index], "" if auto else "Next up,")

    def previous(self) -> str:
        with self._lock:
            if not self.queue:
                return "Nothing is queued."
            self.index = max(0, self.index - 1)
            return self._start(self.queue[self.index], "Going back to")

    def now(self) -> str:
        p = self.now_playing
        if p is None or not self.is_playing():
            return "Nothing is playing right now."
        folder = os.path.basename(os.path.dirname(p))
        where = f" from {folder}" if folder and os.path.dirname(p) != self.music_dir else ""
        return f"This is {self._label(p)}{where}."

    def _start(self, path: str, prefix: str) -> str:
        """Replace the player with ``path`` (caller holds the lock)."""
        self._kill_current()
        self._gen += 1
        gen = self._gen
        try:
            proc = subprocess.Popen(self.player_cmd + [path], env=player_env(self.sink),
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            return f"The music player {self.player_cmd[0]} isn't installed."
        self._proc, self.now_playing = proc, path
        threading.Thread(target=self._watch, args=(proc, gen), daemon=True, name="music-watch").start()
        if self._ducked:
            threading.Thread(target=self._set_stream_volume, args=(proc, self.duck_level), daemon=True).start()
        return f"{prefix} {self._label(path)}.".strip() if prefix else ""

    def _watch(self, proc: subprocess.Popen, gen: int) -> None:
        proc.wait()
        with self._lock:
            if gen != self._gen or self._proc is not proc:
                return                                     # replaced by play/next/stop: not our business
            if proc.returncode == 0 and self.continuous and self.queue:
                self.next(auto=True)
            else:
                self._proc, self.now_playing = None, None

    def _kill_current(self) -> None:
        p = self._proc
        self._proc, self.now_playing = None, None
        if p is not None and p.poll() is None:
            try:
                p.terminate()
                p.wait(timeout=2)
            except Exception:
                p.kill()

    def stop(self, quiet: bool = False) -> str:
        with self._lock:
            was = self.is_playing()
            self._gen += 1
            self._kill_current()
            self.queue, self.index = [], -1
        if was:
            return "" if quiet else "Music stopped."
        return "" if quiet else "Nothing is playing."

    def is_playing(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    # -- ducking (the player's own PulseAudio stream) ----------------------------------
    def duck(self) -> None:
        self._ducked = True
        if self.is_playing() and self.duck_level < 1.0:
            threading.Thread(target=self._set_stream_volume, args=(self._proc, self.duck_level), daemon=True).start()

    def unduck(self) -> None:
        self._ducked = False
        if self.is_playing():
            threading.Thread(target=self._set_stream_volume, args=(self._proc, 1.0), daemon=True).start()

    @staticmethod
    def _set_stream_volume(proc: Optional[subprocess.Popen], level: float) -> None:
        """Set the PulseAudio sink-input volume of ffplay's stream (found by its pid). The stream
        appears a moment after the process starts, so retry briefly."""
        if proc is None:
            return
        for _ in range(20):
            try:
                out = subprocess.run(["pactl", "list", "sink-inputs"], capture_output=True, text=True, timeout=5).stdout
            except Exception:
                return
            idx = None
            for block in out.split("Sink Input #")[1:]:
                if re.search(rf'application\.process\.id = "{proc.pid}"', block):
                    idx = block.split("\n", 1)[0].strip()
                    break
            if idx is not None:
                subprocess.run(["pactl", "set-sink-input-volume", idx, f"{int(level * 100)}%"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
                return
            if proc.poll() is not None:
                return
            time.sleep(0.25)


class VolumeSkill:
    """The robot's speaker volume, through PulseAudio on the default sink (on RD: the USB audio
    adapter on the dock, which start_face.sh makes the default).

    Levels are applied at once so the spoken confirmation is heard at the new level. Mute is
    applied through ``defer`` (the node runs it after the reply has been spoken) so "Muting"
    is still audible; without a ``defer`` callable it mutes immediately.
    """

    def __init__(self, sink: str = "@DEFAULT_SINK@", step: int = 10, max_percent: int = 100,
                 defer: Optional[Callable[[Callable[[], None]], None]] = None):
        self.sink_pattern = sink or DEFAULT_TOKEN           # name/fragment; resolved on every call
        self.step = max(1, int(step))
        self.max = max(1, min(int(max_percent), 150))
        self._defer = defer

    # -- pactl ---------------------------------------------------------------
    def available(self) -> bool:
        return shutil.which("pactl") is not None

    @property
    def sink(self) -> str:
        """The speaker's full sink name when it is present, else the current default."""
        return resolve_sink(self.sink_pattern) or DEFAULT_TOKEN

    def _pactl(self, *args: str) -> Optional[str]:
        try:
            r = subprocess.run(["pactl", *args], capture_output=True, text=True, timeout=5)
        except Exception:
            return None
        return r.stdout if r.returncode == 0 else None

    def level(self) -> Optional[int]:
        out = self._pactl("get-sink-volume", self.sink)
        if not out:
            return None
        pcts = [int(x) for x in re.findall(r"(\d+)%", out)]
        return max(pcts) if pcts else None

    def is_muted(self) -> Optional[bool]:
        out = self._pactl("get-sink-mute", self.sink)
        if not out:
            return None
        return "yes" in out.lower()

    def _apply(self, level: Optional[int] = None, mute: Optional[bool] = None) -> bool:
        ok = True
        if mute is not None:
            ok = self._pactl("set-sink-mute", self.sink, "1" if mute else "0") is not None and ok
        if level is not None:
            ok = self._pactl("set-sink-volume", self.sink, f"{int(level)}%") is not None and ok
        return ok

    def _later(self, fn: Callable[[], None]) -> None:
        if self._defer is not None:
            self._defer(fn)
        else:
            fn()

    # -- actions -------------------------------------------------------------
    FAIL = "I couldn't change the volume."

    def set(self, level: int) -> str:
        level = max(0, min(int(level), self.max))
        if level == 0:
            return self.mute()
        if not self._apply(level=level, mute=False):
            return self.FAIL
        if level >= self.max:
            return "Full volume."
        return f"Volume set to {level} percent."

    def change(self, delta: int) -> str:
        cur = self.level()
        if cur is None:
            return self.FAIL
        muted = self.is_muted() or False
        if delta > 0 and muted:
            # "louder" while muted: come back at the old level, plus the step
            new = max(1, min(cur + int(delta), self.max))
            if not self._apply(level=new, mute=False):
                return self.FAIL
            return f"Unmuted, volume is {new} percent."
        new = max(0, min(cur + int(delta), self.max))
        if new == cur:
            if delta > 0:
                return "I'm already at full volume."
            return "I'm already muted." if muted else "The volume is already at zero."
        if new == 0:
            return self.mute()
        if not self._apply(level=new):
            return self.FAIL
        return f"Volume {'up' if delta > 0 else 'down'} to {new} percent."

    def mute(self) -> str:
        if self.is_muted():
            return "I'm already muted."
        self._later(lambda: self._apply(mute=True))
        return "Muting."

    def unmute(self) -> str:
        if self.is_muted() is False and (self.level() or 0) > 0:
            return f"I'm not muted; the volume is {self.level()} percent."
        lvl = self.level() or 0
        if lvl == 0:
            lvl = max(1, self.max // 2)
        if not self._apply(level=lvl, mute=False):
            return self.FAIL
        return f"Unmuted, volume is {lvl} percent."

    def status(self) -> str:
        lvl = self.level()
        if lvl is None:
            return "I can't read the volume right now."
        if self.is_muted():
            return f"I'm muted; the volume is set to {lvl} percent."
        if lvl >= self.max:
            return "I'm at full volume."
        return f"My volume is {lvl} percent."

    def apply(self, action: str, value: Optional[int] = None) -> str:
        """One entry point for rules and the LLM tool: set / up / down / max / min / mute / unmute / status."""
        a = (action or "").strip().lower()
        amount = int(value) if value is not None else None
        if a in ("set", "level"):
            if amount is None:
                return self.status()
            return self.set(amount)
        if a in ("up", "raise", "louder", "increase"):
            return self.change(abs(amount) if amount else self.step)
        if a in ("down", "lower", "quieter", "decrease"):
            return self.change(-(abs(amount) if amount else self.step))
        if a in ("max", "full"):
            return self.set(self.max)
        if a in ("min", "off", "zero"):
            return self.mute()
        if a == "mute":
            return self.mute()
        if a == "unmute":
            return self.unmute()
        if a in ("status", "get", "query"):
            return self.status()
        return f"I don't know the volume action {action!r}."


__all__ = ["ClockSkill", "WeatherSkill", "NewsSkill", "HomeAssistantSkill", "MusicSkill", "VolumeSkill", "_fuzzy_pick"]
