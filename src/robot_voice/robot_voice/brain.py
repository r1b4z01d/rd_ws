"""The assistant's brain: turns a spoken request into a spoken reply.

Two layers:

1. **Rules** — a small set of high-confidence patterns ("turn on the kitchen
   light", "go to the kitchen", "stop", "play music", "what time is it") are
   handled instantly without a network round trip.
2. **Claude** — everything else goes to Claude (``claude-opus-5-5``) with the
   skills exposed as tools, so questions ("what's the weather tomorrow",
   "any news?", "how far is the moon") and loosely phrased commands all work,
   and follow-ups keep a short conversation memory.

Without API credentials the rules still work and open questions get a polite
"my language model isn't connected" answer.
"""

from __future__ import annotations

import json
import os
import re
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime
from typing import Callable, List, Optional, Protocol

from .skills import ClockSkill, HomeAssistantSkill, MusicSkill, NewsSkill, VolumeSkill, WeatherSkill
from .tasks import NoTaskRunner

MODEL = "claude-opus-5-5"
OLLAMA_MODEL = "qwen2.5:7b"

# Tool catalogue in plain JSON schema, used by the local (Ollama) back-end. The Claude
# back-end exposes the same functions through typed @beta_tool wrappers.
TOOL_SPECS = [
    ("get_weather", "Current conditions and forecast for the robot's home location.",
     {"day": {"type": "string", "description": "today or tomorrow"}}, []),
    ("get_news", "Latest news headlines.",
     {"source": {"type": "string", "description": "feed name such as NPR or BBC; empty for the default"},
      "count": {"type": "integer", "description": "how many headlines, 1 to 8"}}, []),
    ("home_assistant", "Control or query a smart-home device through Home Assistant.",
     {"action": {"type": "string", "description": "turn_on, turn_off, toggle, open, close, lock, unlock or status"},
      "target": {"type": "string", "description": "the device as the person said it, e.g. kitchen light"}}, ["action", "target"]),
    ("play_music", "Play music from the robot's local library.",
     {"query": {"type": "string", "description": "song, album or artist words; empty for random"}}, []),
    ("stop_music", "Stop the music.", {}, []),
    ("music_control", "Skip to the next song, go back to the previous one, or say what is playing.",
     {"action": {"type": "string", "description": "next, previous or now_playing"}}, ["action"]),
    ("volume", "Adjust the robot's own speaker volume (not a smart-home device).",
     {"action": {"type": "string", "description": "set, up, down, max, mute, unmute or status"},
      "level": {"type": "integer", "description": "for set: 0 to 100 percent; for up/down: the step in percent (default 10)"}},
     ["action"]),
    ("navigate_to", "Drive the robot to a named place in the house.",
     {"place": {"type": "string", "description": "a known place such as kitchen or charger"}}, ["place"]),
    ("stop_moving", "Stop the robot and cancel navigation.", {}, []),
    ("list_places", "Names of the places the robot can drive to.", {}, []),
    ("run_task", "Do a job with the robot's arm, such as picking an object up. The robot drives to the place the job "
                 "belongs to by itself first, then runs the trained policy.",
     {"name": {"type": "string", "description": "the job's name as listed, e.g. pick up tigger"}}, ["name"]),
    ("list_tasks", "The jobs the robot's arm can do, and where each one is done.", {}, []),
    ("get_time", "The current local time and date.", {}, []),
]


def _claude_credentials_present() -> bool:
    """Cheap check for anything the Anthropic SDK could authenticate with."""
    if os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("ANTHROPIC_AUTH_TOKEN"):
        return True
    return os.path.isdir(os.path.expanduser("~/.config/anthropic"))


class Navigator(Protocol):
    def go_to(self, place: str) -> str: ...
    def stop(self) -> str: ...
    def places(self) -> List[str]: ...
    def status(self) -> str: ...


class TaskRunner(Protocol):
    """Landmark tasks (tasks.py): "pick up tigger" = drive to its landmark, run its policy."""
    busy: bool
    def names(self) -> List[str]: ...
    def match(self, text: str): ...
    def run(self, text: str) -> str: ...
    def stop(self) -> str: ...
    def status(self) -> str: ...


class NoNavigator:
    def go_to(self, place: str) -> str:
        return "I can't drive right now; navigation isn't connected."

    def stop(self) -> str:
        return "I'm not moving."

    def places(self) -> List[str]:
        return []

    def status(self) -> str:
        return "Navigation isn't connected."


# ---------------------------------------------------------------------------
# Rule-based fast path
# ---------------------------------------------------------------------------
_R_LIGHT = re.compile(r"^(?:please\s+)?(?:can you\s+|could you\s+|would you\s+)?(turn|switch|shut|put)\s+(on|off)\s+(?:the\s+)?(.+?)(?:\s+please)?[.!?]*$")
_R_LIGHT2 = re.compile(r"^(?:please\s+)?(?:can you\s+|could you\s+)?(turn|switch)\s+(?:the\s+)?(.+?)\s+(on|off)(?:\s+please)?[.!?]*$")
_R_TOGGLE = re.compile(r"^(?:please\s+)?toggle\s+(?:the\s+)?(.+?)[.!?]*$")
_R_GO = re.compile(r"^(?:please\s+)?(?:can you\s+|could you\s+)?(?:go|move|drive|head|navigate|come|return|travel|roll)\s+(?:on\s+)?(?:back\s+|over\s+|down\s+|up\s+)?(?:to\s+|towards?\s+|into\s+)?(?:the\s+|your\s+|my\s+)?(.+?)(?:\s+please)?[.!?]*$")
_R_STOP = re.compile(r"^(?:please\s+)?(stop|halt|freeze|stay|wait|hold on|cancel)(?:\s+(moving|there|right there|navigation|the music|music|playing))?(?:\s+please)?[.!?]*$")
_PRE_ANY = r"^(?:please\s+)?(?:can you\s+|could you\s+|would you\s+|will you\s+|hey\s+)?"
_POST_ANY = r"(?:\s+please)?[.!?]*$"
_R_NEXT = re.compile(_PRE_ANY + r"(?:skip(?:\s+(?:this|that|the|it))?(?:\s+(?:song|track|one|music))?|(?:play\s+(?:the\s+)?)?next(?:\s+(?:song|track|one))?"
                     r"|(?:go\s+to\s+the\s+)?next\s+(?:song|track)|play\s+(?:something|anything)\s+else|play\s+(?:another|a\s+different)(?:\s+(?:song|track|one))?"
                     r"|change\s+the\s+(?:song|track|music)|different\s+song|another\s+song|not\s+this\s+(?:one|song))" + _POST_ANY)
_R_PREV = re.compile(_PRE_ANY + r"(?:(?:go\s+)?back(?:\s+(?:one|a)\s+(?:song|track))?|(?:play\s+(?:the\s+)?)?(?:previous|last)\s+(?:song|track|one)(?:\s+again)?"
                     r"|play\s+that\s+(?:one\s+)?again|(?:replay|restart|start\s+over)(?:\s+(?:this|that|the)\s+(?:song|track|one))?)" + _POST_ANY)
_R_NOW = re.compile(_PRE_ANY + r"(?:what(?:'s| is)\s+(?:this|playing|this song|the song|that song|this track|that|on)(?:\s+(?:song|called|now))?|what\s+(?:song|track)\s+is\s+(?:this|that|playing)"
                    r"|who\s+(?:is\s+this|sings\s+this|is\s+singing|plays\s+this)|what\s+am\s+i\s+listening\s+to|now\s+playing|name\s+(?:this|that|the)\s+(?:song|track))" + _POST_ANY)
_PLAYING_ONLY = re.compile(r"^(?:skip|next|back|go back|play something else|play anything else|another song|what is this|what's this|who is this|what's that|what is that|what's on|what is on|start over|replay)[.!?]*$")
_R_PLAY = re.compile(r"^(?:please\s+)?(?:can you\s+)?play\s+(?:some\s+|me\s+some\s+)?(.*?)(?:\s+please)?[.!?]*$")
# --- speaker volume -----------------------------------------------------------------
_VOL = r"(?:the\s+|your\s+|my\s+)?(?:speaker\s+|sound\s+|audio\s+|voice\s+|music\s+|tts\s+)?volume"
_VOL_IT = r"(?:it|yourself|" + _VOL + r"|(?:the\s+|your\s+)?(?:sound|audio|speaker|music|speakers))"
_LEVEL = (r"(max(?:imum)?|full|highest|loudest|top|min(?:imum)?|lowest|quietest|zero|off|half(?:way)?|medium|middle"
          r"|(?:a\s+)?quarter|one hundred|a hundred|hundred"
          r"|(?:ten|twenty|thirty|forty|fifty|sixty|seventy|eighty|ninety)(?:[- ](?:one|two|three|four|five|six|seven|eight|nine))?"
          r"|\d{1,3})")
_PCT = r"(?:\s*(?:percent|%))?"
_AMOUNT = r"(?:\s+(?:by\s+(\d{1,3})" + _PCT + r"|(a\s+(?:bit|little|touch|tad|notch)|slightly|a\s+(?:lot|bunch)|much|way)))?"
_PRE = r"^(?:please\s+)?(?:can you\s+|could you\s+|would you\s+|will you\s+)?"
_POST = r"(?:\s+please)?[.!?]*$"
_R_VOL_SET = re.compile(_PRE + r"(?:set|put|change|turn|make|adjust|bring|crank|move)\s+" + _VOL
                        + r"\s+(?:up\s+|down\s+)?(?:to|at|on)\s+(?:the\s+)?" + _LEVEL + _PCT + _POST)
_R_VOL_IS = re.compile(_PRE + _VOL + r"\s+(?:to|at|on|is|=|equals)?\s*(?:the\s+)?" + _LEVEL + _PCT + _POST)
_R_LEVEL_VOL = re.compile(_PRE + r"(?:go\s+(?:to\s+)?|at\s+|on\s+)?" + _LEVEL + _PCT + r"\s+volume" + _POST)
_R_VOL_TURN = re.compile(_PRE + r"(?:turn|crank|bring|pump|bump|take|put)\s+" + _VOL_IT
                         + r"\s+(?:(all\s+the\s+way|way|a\s+(?:bit|little|touch|tad|notch)|slightly|a\s+lot|much)\s+)?(up|down)"
                         + _AMOUNT + _POST)
_R_VOL_RAISE = re.compile(_PRE + r"(raise|increase|boost|up|lower|decrease|reduce|drop|cut)\s+" + _VOL + _AMOUNT + _POST)
_R_VOL_UPDOWN = re.compile(_PRE + _VOL + r"\s+(up|down)" + _AMOUNT + _POST)
_R_VOL_LOUDER = re.compile(_PRE + r"(?:(a\s+(?:bit|little|touch|tad)|slightly|a\s+lot|much|way)\s+)?(louder|quieter|softer)" + _POST)
_R_VOL_SPEAK = re.compile(_PRE + r"(?:speak|talk)\s+(up|louder|softer|quieter|more\s+quietly|more\s+loudly)" + _POST)
_R_VOL_TOO = re.compile(r"^(?:(?:you're|you are|that's|that is|it's|it is|this is)\s+)?(?:(?:a\s+(?:bit|little)|way|much)\s+)?too\s+(loud|quiet|soft)" + _POST
                        + r"|^not\s+so\s+(loud|quiet)" + _POST)
_R_VOL_MUTE = re.compile(_PRE + r"(mute|unmute|silence)(?:\s+" + _VOL_IT + r")?" + _POST)
_R_VOL_SOUND = re.compile(_PRE + r"(?:turn|switch)\s+(?:(on|off)\s+)?(?:your\s+(?:sound|audio|speaker|speakers|voice|volume)|the\s+(?:sound|audio|volume))(?:\s+(?:back\s+)?(on|off))?" + _POST)
_R_VOL_STATUS = re.compile(_PRE + r"(?:(?:what's|what is|whats|tell me)\s+(?:the\s+|your\s+)?(?:current\s+)?volume(?:\s+(?:level|at|set to|setting))?"
                           r"|how\s+loud\s+(?:are\s+you|is\s+it|is\s+(?:the|your)\s+(?:volume|sound|speaker))|volume\s+(?:level|status|check)|check\s+(?:the\s+|your\s+)?volume)" + _POST)
_TENS = {"ten": 10, "twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90}
_ONES = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9}


def _level_value(word: str) -> Optional[int]:
    w = re.sub(r"\s+", " ", (word or "").strip().lower())
    if w.isdigit():
        return int(w)
    if w in ("max", "maximum", "full", "highest", "loudest", "top", "one hundred", "a hundred", "hundred"):
        return 100
    if w in ("min", "minimum", "lowest", "quietest", "zero", "off"):
        return 0
    if w in ("half", "halfway", "medium", "middle"):
        return 50
    if w in ("quarter", "a quarter"):
        return 25
    parts = re.split(r"[- ]", w)
    if parts and parts[0] in _TENS:
        return _TENS[parts[0]] + (_ONES.get(parts[1], 0) if len(parts) > 1 else 0)
    return None


def _amount_value(by: Optional[str], word: Optional[str]) -> Optional[int]:
    if by:
        return int(by)
    w = re.sub(r"\s+", " ", (word or "").strip().lower())
    if not w:
        return None
    if w in ("a lot", "a bunch", "much", "way"):
        return 20
    return 5          # a bit, a little, a touch, a tad, a notch, slightly


_R_LOOSE_UP = re.compile(r"\b(louder|speak up|turn (?:it|you|yourself) up|crank (?:it|you) up|can'?t hear you|cannot hear you|barely hear you|hard(?:ly)? (?:to )?hear you|too quiet|too soft|not loud enough)\b")
_R_LOOSE_DOWN = re.compile(r"\b(quieter|softer|turn (?:it|you|yourself) down|too loud|not so loud|way too loud|hurting my ears|lower (?:it|your voice))\b")
_R_LOOSE_LEVEL = re.compile(r"\bvolume\b.*?\b" + _LEVEL + _PCT + r"\b|\b" + _LEVEL + _PCT + r"\s+volume\b")


def parse_volume_loose(t: str) -> Optional[tuple]:
    """Looser volume intent for the fallback paths (small local model, no model): keyword based,
    only used after the exact rules and only when the sentence is clearly about the robot's sound."""
    strict = parse_volume(t)
    if strict is not None:
        return strict
    if re.search(r"\b(light|lights|lamp|fan|plug|switch|thermostat|heat|music|song|tv|television)\b", t):
        return None                                   # about a device or the music, not the speaker
    if re.search(r"\bunmute\b", t):
        return ("unmute", None)
    if re.search(r"\bmute\b", t):
        return ("mute", None)
    m = _R_LOOSE_LEVEL.search(t)
    if m and "volume" in t:
        lvl = _level_value(m.group(1) or m.group(2))
        if lvl is not None:
            return ("set", lvl)
    if _R_LOOSE_UP.search(t) or re.search(r"\bvolume\b.*\b(up|higher|louder|more)\b", t):
        return ("up", None)
    if _R_LOOSE_DOWN.search(t) or re.search(r"\bvolume\b.*\b(down|lower|quieter|less)\b", t):
        return ("down", None)
    return None


def parse_volume(t: str) -> Optional[tuple]:
    """Speaker-volume intent in normalised text, or None.

    Returns (action, value): ("set", percent) | ("up"|"down", step or None) | ("mute"|"unmute"|"status", None).
    """
    m = _R_VOL_SET.match(t) or _R_VOL_IS.match(t) or _R_LEVEL_VOL.match(t)
    if m:
        lvl = _level_value(m.group(1))
        return ("set", lvl) if lvl is not None else None
    m = _R_VOL_TURN.match(t)
    if m:
        qual, direction = (m.group(1) or ""), m.group(2)
        if qual.startswith("all") or qual == "way":
            return ("set", 100 if direction == "up" else 0)
        return (direction, _amount_value(m.group(3), m.group(4) or qual))
    m = _R_VOL_RAISE.match(t)
    if m:
        up = m.group(1) in ("raise", "increase", "boost", "up")
        return ("up" if up else "down", _amount_value(m.group(2), m.group(3)))
    m = _R_VOL_UPDOWN.match(t)
    if m:
        return (m.group(1), _amount_value(m.group(2), m.group(3)))
    m = _R_VOL_LOUDER.match(t)
    if m:
        return ("up" if m.group(2) == "louder" else "down", _amount_value(None, m.group(1)))
    m = _R_VOL_SPEAK.match(t)
    if m:
        return ("up" if m.group(1) in ("up", "louder", "more loudly") else "down", None)
    m = _R_VOL_TOO.match(t)
    if m:
        word = m.group(1) or m.group(2)
        return ("down" if word == "loud" else "up", None)
    m = _R_VOL_MUTE.match(t)
    if m:
        return ("unmute" if m.group(1) == "unmute" else "mute", None)
    m = _R_VOL_SOUND.match(t)
    if m:
        state = m.group(1) or m.group(2)
        if state:
            return ("unmute" if state == "on" else "mute", None)
    if _R_VOL_STATUS.match(t):
        return ("status", None)
    return None


_R_TIME = re.compile(r"^(?:what(?:'s| is)\s+the\s+time|what time is it)(?:\s+now)?[?.!]*$")
_R_DATE = re.compile(r"^(?:what(?:'s| is)\s+(?:the\s+date|today(?:'s date)?)|what day is it)[?.!]*$")
_R_WEATHER = re.compile(
    r"^(?:.*\b(?:weather|forecast|temperature|how (?:hot|cold|warm) is it|rain(?:ing|y)?|snow(?:ing)?|sunny|umbrella|jacket)\b.*)$"
)
_R_WEATHER_DAY = re.compile(r"\b(tomorrow|tonight|today|right now|now|this (?:morning|afternoon|evening|weekend))\b")
_R_NEWS = re.compile(
    r"^(?:.*\b(?:news|headlines?|what(?:'s| is) (?:going on|happening) in the world)\b.*)$"
)
_R_NEWS_COUNT = re.compile(r"\b(one|two|three|four|five|six|\d)\b")


class Brain:
    def __init__(
        self,
        weather: WeatherSkill,
        news: NewsSkill,
        home: HomeAssistantSkill,
        music: MusicSkill,
        clock: ClockSkill,
        navigator: Optional[Navigator] = None,
        volume: Optional[VolumeSkill] = None,
        tasks: Optional[TaskRunner] = None,
        persona: str = "Robot Disco",
        location_name: str = "",
        model: str = MODEL,
        llm_enabled: bool = True,
        llm_backend: str = "auto",            # auto | claude | ollama | none
        ollama_url: str = "http://127.0.0.1:11434",
        ollama_model: str = OLLAMA_MODEL,
        max_history: int = 8,
        history_ttl: float = 600.0,
        log: Optional[Callable[[str], None]] = None,
        rules_first: bool = True,
    ):
        self.weather, self.news, self.home, self.music, self.clock = weather, news, home, music, clock
        self.volume = volume or VolumeSkill()
        self.nav: Navigator = navigator or NoNavigator()
        self.tasks: TaskRunner = tasks or NoTaskRunner()
        self.persona = persona
        self.location_name = location_name
        self.model = model
        self.rules_first = rules_first
        self._log = log or (lambda m: None)
        self._history: List[dict] = []
        self._history_time = 0.0
        self.max_history = max_history
        self.history_ttl = history_ttl
        self._lock = threading.Lock()
        self._client = None
        self._tools = None
        self.llm_reason = ""
        self.llm_backend = (llm_backend or "auto").lower()
        self.ollama_urls = [u.strip().rstrip("/") for u in str(ollama_url).split(",") if u.strip()]
        self.ollama_url = self.ollama_urls[0] if self.ollama_urls else ""
        self.ollama_model = ollama_model
        self._ollama_ok = False
        self._ollama_last_probe = 0.0
        if llm_enabled and self.llm_backend != "none":
            self._init_llm()

    # ------------------------------------------------------------------
    # LLM setup
    # ------------------------------------------------------------------
    def _init_llm(self) -> None:
        want_claude = self.llm_backend in ("auto", "claude")
        want_ollama = self.llm_backend in ("auto", "ollama")
        if want_claude:
            if self.llm_backend == "claude" or _claude_credentials_present():
                try:
                    import anthropic
                    self._client = anthropic.Anthropic(timeout=25.0, max_retries=1)
                    self._tools = self._build_tools()
                except ImportError:
                    self.llm_reason = "the anthropic package is not installed"
                except Exception as e:   # no credentials anywhere
                    self.llm_reason = f"no Claude credentials ({type(e).__name__})"
                    self._client = None
            else:
                self.llm_reason = "no Claude API key is configured"
        if want_ollama and self._client is None:
            self._ollama_ok = self._probe_ollama()
            if not self._ollama_ok and not self.llm_reason:
                self.llm_reason = f"no local language model at {self.ollama_url}"

    def _probe_ollama(self) -> bool:
        """Try each configured Ollama server in order (e.g. the GPU box, then the robot
        itself); the first one that answers and has the model wins."""
        self._ollama_last_probe = time.time()
        missing = []
        for url in self.ollama_urls:
            try:
                with urllib.request.urlopen(f"{url}/api/tags", timeout=1.5) as r:
                    tags = json.loads(r.read().decode("utf-8"))
            except Exception as e:
                self._log(f"Ollama not reachable at {url}: {type(e).__name__}")
                continue
            names = {m.get("name", "") for m in tags.get("models", [])}
            base = self.ollama_model.split(":")[0]
            if self.ollama_model in names or any(n.split(":")[0] == base for n in names):
                self.ollama_url = url
                self._log(f"Ollama ready at {url} with {self.ollama_model}")
                threading.Thread(target=self._warm_ollama, args=(url,), daemon=True).start()
                return True
            missing.append(url)
            self._log(f"Ollama at {url} lacks {self.ollama_model!r} (have: {', '.join(sorted(names)) or 'none'}); "
                      f"run there: ollama pull {self.ollama_model}")
        self.llm_reason = (f"the local model {self.ollama_model} is not downloaded" if missing
                           else f"no language model server at {', '.join(self.ollama_urls)}")
        return False

    def _warm_ollama(self, url: str) -> None:
        """Load the model into the server's memory now (a cold load took ~28 s on the
        GPU box) so the first spoken question doesn't pay for it."""
        body = {"model": self.ollama_model, "messages": [], "keep_alive": "24h"}
        req = urllib.request.Request(f"{url}/api/chat", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(req, timeout=180.0):
                pass
            self._log(f"Ollama model {self.ollama_model} warmed at {url} in {time.perf_counter() - t0:.1f}s")
        except Exception as e:
            self._log(f"Ollama warm-up at {url} failed: {type(e).__name__}")

    @property
    def active_backend(self) -> str:
        if self._client is not None:
            return f"claude ({self.model})"
        if self._ollama_ok:
            return f"ollama ({self.ollama_model} at {self.ollama_url})"
        return "rules only"

    @property
    def llm_available(self) -> bool:
        return self._client is not None or self._ollama_ok

    def _build_tools(self):
        from anthropic import beta_tool
        brain = self

        @beta_tool
        def get_weather(day: str = "today") -> str:
            """Current conditions and forecast for the robot's home location.

            Args:
                day: "today" or "tomorrow".
            """
            return brain.weather.report(day)

        @beta_tool
        def get_news(source: str = "", count: int = 4) -> str:
            """Latest news headlines.

            Args:
                source: Preferred feed name (for example NPR or BBC); empty for the default.
                count: How many headlines to read, 1 to 8.
            """
            return brain.news.headlines(source, count)

        @beta_tool
        def home_assistant(action: str, target: str) -> str:
            """Control or query a smart-home device through Home Assistant.

            Args:
                action: One of turn_on, turn_off, toggle, open, close, lock, unlock, or status.
                target: The device as the person said it, e.g. "kitchen light" or "garage door".
            """
            if action.lower() == "status":
                return brain.home.state_of(target)
            return brain.home.turn(target, action)

        @beta_tool
        def play_music(query: str = "") -> str:
            """Play music from the robot's local library.

            Args:
                query: Song, album or artist words to match; empty plays something at random.
            """
            return brain.music.play(query)

        @beta_tool
        def stop_music() -> str:
            """Stop the music that is playing."""
            return brain.music.stop()

        @beta_tool
        def music_control(action: str) -> str:
            """Skip to the next song, go back to the previous one, or say what is playing.

            Args:
                action: One of next, previous, or now_playing.
            """
            return brain.music_action(action)

        @beta_tool
        def volume(action: str, level: int = 0) -> str:
            """Adjust the robot's own speaker volume (not a smart-home device).

            Args:
                action: One of set, up, down, max, mute, unmute, or status.
                level: For set, the target 0 to 100 percent; for up or down, the step in percent (0 means the default step).
            """
            return brain.volume.apply(action, level if level > 0 or action == "set" else None)

        @beta_tool
        def navigate_to(place: str) -> str:
            """Drive the robot to a named place in the house.

            Args:
                place: One of the known places (see list_places), e.g. "kitchen" or "charger".
            """
            return brain.nav.go_to(place)

        @beta_tool
        def stop_moving() -> str:
            """Stop the robot where it is and cancel navigation."""
            return brain.nav.stop()

        @beta_tool
        def list_places() -> str:
            """Names of the places the robot can drive to."""
            places = brain.nav.places()
            return ", ".join(places) if places else "no places are configured"

        @beta_tool
        def run_task(name: str) -> str:
            """Do a job with the robot's arm, such as picking an object up. The robot drives to the
            place the job belongs to by itself first, then runs the trained policy there.

            Args:
                name: The job's name as listed by list_tasks, e.g. "pick up tigger".
            """
            return brain.tasks.run(name)

        @beta_tool
        def list_tasks() -> str:
            """The jobs the robot's arm can do, and where each one is done."""
            names = brain.tasks.names()
            return ", ".join(names) if names else "no arm jobs are set up"

        @beta_tool
        def get_time() -> str:
            """The current local time and date."""
            return brain.clock.time() + " " + brain.clock.date()

        return [get_weather, get_news, home_assistant, play_music, stop_music, music_control, volume,
                navigate_to, stop_moving, list_places, run_task, list_tasks, get_time]

    def _system_prompt(self, person: Optional[str]) -> str:
        places = self.nav.places()
        devices = self.home.names(limit=30) if self.home.available() else []
        now = datetime.now().strftime("%A %B %-d, %Y, %-I:%M %p")
        lines = [
            f"You are {self.persona}, a friendly household robot with a screen face, wheels, an arm, and a speaker. "
            "You are talking out loud through text-to-speech, so answer the way a person speaks: one or two short "
            "sentences, plain words, no lists, no markdown, no emoji, no URLs. Spell out units. "
            "Tools exist only for live facts about this house and robot: the weather here, today's news headlines, "
            "smart-home devices, the music player (play, stop, next, previous, what is playing), your own speaker volume, driving to a place, "
            "jobs for your arm (run_task: you drive to the job's place first, then do it), and the current time. For those, call the tool "
            "and report its result; never invent a forecast, a headline or a device state. Everything else — science, "
            "history, geography, how things work, recommendations, opinions, small talk — you answer directly from "
            "your own knowledge in one or two sentences (for example, the Moon is about 239,000 miles away). "
            "Never say you lack a tool for a general question. If you truly cannot do something, say so briefly.",
            f"Current date and time: {now}.",
        ]
        if self.location_name:
            lines.append(f"You live in {self.location_name}.")
        if person:
            lines.append(f"The person talking to you is {person}; use their name occasionally, not every turn.")
        if places:
            lines.append("Places you can drive to: " + ", ".join(places) + ". 'Charger', 'dock' and 'home' all mean the charger.")
        else:
            lines.append("Navigation is not available right now.")
        tasks = self.tasks.names()
        if tasks:
            lines.append("Jobs your arm can do, each at its own place: " + "; ".join(tasks) +
                         ". When asked for one, call run_task with the job's name exactly as listed and report its reply; "
                         "you drive to the place by yourself first." + (" You are busy with a job right now." if self.tasks.busy else ""))
        if devices:
            lines.append("Smart-home devices you can control: " + ", ".join(devices) + ".")
        elif not self.home.available():
            lines.append("Home Assistant is not connected, so you cannot control lights or switches.")
        if not self.weather.available():
            lines.append("Your weather location is not configured.")
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Public entry
    # ------------------------------------------------------------------
    def handle(self, text: str, person: Optional[str] = None) -> str:
        text = (text or "").strip()
        if not text:
            return "Yes?"
        with self._lock:
            if self.rules_first:
                quick = self.rules(text)
                if quick is not None:
                    self._log(f"rule: {text!r} -> {quick!r}")
                    return quick
            if self._client is not None:
                reply = self._ask_claude(text, person)
                if reply:
                    return reply
                if self._client is None and self.llm_backend == "auto" and not self._ollama_ok:
                    self._ollama_ok = self._probe_ollama()      # Claude just got disabled: try local
            if self._client is None and not self._ollama_ok and self.ollama_urls \
                    and self.llm_backend in ("auto", "ollama") and time.time() - self._ollama_last_probe > 30.0:
                self._ollama_ok = self._probe_ollama()          # a server may have come up since
            if self._client is None and self._ollama_ok:
                # Small local models sometimes narrate instead of calling a tool; serve the
                # unambiguous skill intents directly and let the model handle the rest.
                quick = self.rules(text, loose=True)
                if quick is not None:
                    self._log(f"rule (local-model guard): {text!r} -> {quick!r}")
                    return quick
                reply = self._ask_ollama(text, person)
                if reply:
                    return reply
            fallback = self.rules(text, loose=True)
            if fallback is not None:
                return fallback
            if not self.llm_available:
                return f"I can do lights, music, the weather, the news and driving around, but I can't answer that: {self.llm_reason}."
            return "Sorry, I didn't catch that."

    # ------------------------------------------------------------------
    # Rules
    # ------------------------------------------------------------------
    def rules(self, text: str, loose: bool = False) -> Optional[str]:
        t = re.sub(r"\s+", " ", text.lower().strip())
        t = re.sub(r"^(hey|ok|okay|please|so|um|uh)[, ]+", "", t)
        for rx, action in ((_R_NEXT, "next"), (_R_PREV, "previous"), (_R_NOW, "now_playing")):
            if rx.match(t) and (self.music.is_playing() or not _PLAYING_ONLY.match(t)):
                return self.music_action(action)
        m = _R_STOP.match(t)
        if m:
            what = m.group(2) or ""
            if "music" in what or "playing" in what:
                return self.music.stop()
            if self.tasks.busy:
                r = self.tasks.stop()           # ends the drive or the policy run (arm goes home)
                nav = self.nav.stop()
                return r or nav
            if self.music.is_playing() and not what:
                r = self.music.stop()
                nav = self.nav.stop()
                return f"{r} {nav}".strip()
            return self.nav.stop()
        vol = parse_volume(t)
        if vol is not None:
            return self.volume.apply(vol[0], vol[1])
        m = _R_LIGHT.match(t) or None
        if m:
            return self.home.turn(m.group(3), m.group(2))
        m = _R_LIGHT2.match(t)
        if m:
            return self.home.turn(m.group(2), m.group(3))
        m = _R_TOGGLE.match(t)
        if m:
            return self.home.turn(m.group(1), "toggle")
        m = _R_PLAY.match(t)
        if m:
            q = m.group(1).strip()
            q = re.sub(r"^(music|a song|something|songs|anything)$", "", q).strip()
            q = re.sub(r"^(music|songs?)\s+(by|from)\s+", "", q)
            return self.music.play(q)
        m = _R_TIME.match(t)
        if m:
            return self.clock.time()
        m = _R_DATE.match(t)
        if m:
            return self.clock.date()
        hit = self.tasks.match(t)             # "pick up tigger" = a landmark task: drive there, run its policy
        if hit is not None:
            return self.tasks.start(*hit)
        m = _R_GO.match(t)
        if m:
            place = m.group(1).strip()
            place = re.sub(r"^(your|the|my)\s+", "", place)
            known = self.nav.places()
            hit = _match_place(place, known) if known else None
            if hit:
                return self.nav.go_to(hit)
            if loose:
                return self.nav.go_to(place)
        if loose:
            vol = parse_volume_loose(t)
            if vol is not None:
                return self.volume.apply(vol[0], vol[1])
            if self.music.is_playing():
                if re.search(r"\b(skip|next)\b", t):
                    return self.music_action("next")
                if re.search(r"\b(previous|last song|go back|again)\b", t):
                    return self.music_action("previous")
                if re.search(r"\bwhat\b.*\b(song|playing|track|this)\b|\bwho\b.*\b(this|singing|sings)\b", t):
                    return self.music_action("now_playing")
            if _R_WEATHER.match(t):
                d = _R_WEATHER_DAY.search(t)
                return self.weather.report("tomorrow" if d and d.group(1) == "tomorrow" else "today")
            if _R_NEWS.match(t):
                m = _R_NEWS_COUNT.search(t)
                words = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6}
                n = words.get(m.group(1), None) if m else None
                if n is None and m and m.group(1).isdigit():
                    n = int(m.group(1))
                return self.news.headlines(count=n or 4)
        return None

    def music_action(self, action: str) -> str:
        a = (action or "").strip().lower().replace(" ", "_")
        if a in ("next", "skip", "next_song"):
            r = self.music.next()
            return r or "Nothing is playing; say play some music."
        if a in ("previous", "back", "previous_song", "last"):
            return self.music.previous()
        if a in ("now_playing", "now", "what", "current"):
            return self.music.now()
        if a == "stop":
            return self.music.stop()
        return f"I don't know the music action {action!r}."

    # ------------------------------------------------------------------
    # Tool dispatch (local back-end)
    # ------------------------------------------------------------------
    def _dispatch(self, name: str, args: dict) -> str:
        a = args or {}
        try:
            if name == "get_weather":
                return self.weather.report(str(a.get("day", "today")))
            if name == "get_news":
                return self.news.headlines(str(a.get("source", "")), int(a.get("count", 4) or 4))
            if name == "home_assistant":
                action, target = str(a.get("action", "")), str(a.get("target", ""))
                return self.home.state_of(target) if action.lower() == "status" else self.home.turn(target, action)
            if name == "play_music":
                return self.music.play(str(a.get("query", "")))
            if name == "stop_music":
                return self.music.stop()
            if name == "music_control":
                return self.music_action(str(a.get("action", "")))
            if name == "volume":
                lvl = a.get("level")
                try:
                    lvl = int(lvl) if lvl not in (None, "") else None
                except (TypeError, ValueError):
                    lvl = None
                return self.volume.apply(str(a.get("action", "status")), lvl)
            if name == "navigate_to":
                return self.nav.go_to(str(a.get("place", "")))
            if name == "stop_moving":
                return self.nav.stop()
            if name == "list_places":
                return ", ".join(self.nav.places()) or "no places are configured"
            if name == "run_task":
                return self.tasks.run(str(a.get("name", "")))
            if name == "list_tasks":
                return ", ".join(self.tasks.names()) or "no arm jobs are set up"
            if name == "get_time":
                return self.clock.time() + " " + self.clock.date()
        except Exception as e:
            return f"error: {e}"
        return f"unknown tool {name}"

    @staticmethod
    def _ollama_tools() -> List[dict]:
        tools = []
        for name, desc, props, required in TOOL_SPECS:
            tools.append({"type": "function", "function": {
                "name": name, "description": desc,
                "parameters": {"type": "object", "properties": props, "required": required}}})
        return tools

    def _ask_ollama(self, text: str, person: Optional[str]) -> Optional[str]:
        """Local model through Ollama's chat API with tool calling (a few rounds max)."""
        if time.time() - self._history_time > self.history_ttl:
            self._history = []
        messages = [{"role": "system", "content": self._system_prompt(person)}] + self._history \
                   + [{"role": "user", "content": text}]
        t0 = time.perf_counter()
        for _round in range(4):
            body = {"model": self.ollama_model, "messages": messages, "tools": self._ollama_tools(),
                    "stream": False, "keep_alive": "24h",
                    "options": {"temperature": 0.3, "num_predict": 220}}
            req = urllib.request.Request(f"{self.ollama_url}/api/chat", data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"}, method="POST")
            try:
                with urllib.request.urlopen(req, timeout=90.0) as r:
                    data = json.loads(r.read().decode("utf-8"))
            except urllib.error.URLError as e:
                self._log(f"Ollama request to {self.ollama_url} failed: {e}")
                self._ollama_ok = self._probe_ollama()           # fail over (GPU box -> robot)
                if self._ollama_ok and _round == 0:
                    continue
                return "I can't reach my language model right now."
            except Exception as e:
                self._log(f"Ollama failed: {type(e).__name__}: {e}")
                return None
            msg = data.get("message", {}) or {}
            calls = msg.get("tool_calls") or []
            if not calls:
                reply = re.sub(r"\s+", " ", (msg.get("content") or "")).strip()
                reply = re.sub(r"<think>.*?</think>", "", reply).strip()
                if reply:
                    self._history = (self._history + [{"role": "user", "content": text},
                                                      {"role": "assistant", "content": reply}])[-self.max_history:]
                    self._history_time = time.time()
                self._log(f"ollama {time.perf_counter() - t0:.1f}s: {text!r} -> {reply!r}")
                return reply or None
            messages.append(msg)
            for call in calls:
                fn = call.get("function", {}) or {}
                name, args = fn.get("name", ""), fn.get("arguments", {}) or {}
                if isinstance(args, str):
                    try:
                        args = json.loads(args)
                    except json.JSONDecodeError:
                        args = {}
                result = self._dispatch(name, args)
                self._log(f"ollama tool {name}({args}) -> {result[:80]!r}")
                messages.append({"role": "tool", "tool_name": name, "content": result})
        return "That took too many steps; try asking in a simpler way."

    # ------------------------------------------------------------------
    # Claude
    # ------------------------------------------------------------------
    def _ask_claude(self, text: str, person: Optional[str]) -> Optional[str]:
        import anthropic

        if time.time() - self._history_time > self.history_ttl:
            self._history = []
        messages = self._history + [{"role": "user", "content": text}]
        kwargs = dict(
            model=self.model,
            max_tokens=600,                      # spoken replies are short by design
            system=self._system_prompt(person),
            tools=self._tools,
            messages=messages,
            output_config={"effort": "low"},     # chat / command routing: low effort is fastest
        )
        t0 = time.perf_counter()
        try:
            try:
                runner = self._client.beta.messages.tool_runner(
                    betas=["server-side-fallback-2026-07-01"], fallbacks="default", **kwargs)
                final = self._drain(runner, messages)
            except TypeError:
                # older SDK without the fallbacks parameter
                runner = self._client.beta.messages.tool_runner(**kwargs)
                final = self._drain(runner, messages)
        except anthropic.AuthenticationError:
            self._log("Claude: authentication failed; disabling the language model")
            self._client, self.llm_reason = None, "my Claude API key was rejected"
            return None
        except anthropic.RateLimitError:
            return "I'm being rate limited right now; try again in a moment."
        except anthropic.APIStatusError as e:
            self._log(f"Claude API error {e.status_code}: {e.message}")
            return None
        except anthropic.APIConnectionError:
            self._log("Claude: connection error")
            return "I can't reach my language model right now."
        except TypeError as e:
            # anthropic 2.x constructs the client happily and only complains here when no
            # api key / auth token / profile could be found
            if "authentication" in str(e).lower() or "api_key" in str(e):
                self._log("Claude: no credentials found; disabling the language model")
                self._client, self.llm_reason = None, "no Claude API key is configured"
                return None
            self._log(f"Claude failed: {type(e).__name__}: {e}")
            return None
        except Exception as e:
            self._log(f"Claude failed: {type(e).__name__}: {e}")
            return None
        self._log(f"claude {time.perf_counter() - t0:.1f}s: {text!r} -> {final!r}")
        return final

    def _drain(self, runner, messages: List[dict]) -> Optional[str]:
        last = None
        for message in runner:
            last = message
            messages.append({"role": "assistant", "content": message.content})
            tool_response = runner.generate_tool_call_response()
            if tool_response is not None:
                messages.append(tool_response)
        if last is None:
            return None
        if last.stop_reason == "refusal":
            return "I'd rather not answer that one."
        text = " ".join(b.text for b in last.content if b.type == "text").strip()
        text = re.sub(r"\s+", " ", text)
        if text:
            # remember the exchange (trimmed) for follow-ups
            self._history = messages[-self.max_history:]
            self._history_time = time.time()
        return text or None


def _match_place(query: str, places: List[str]) -> Optional[str]:
    """Map what was said ("the couch", "back home", "dining room") to a known place name."""
    from .skills import _fuzzy_pick
    q = re.sub(r"^(?:the|your|my|our)\s+", "", query.lower().strip())
    lower = {p.lower(): p for p in places}
    for alias, canon in (("dock", "charger"), ("home", "charger"), ("base", "charger"),
                         ("charging station", "charger"), ("living room", "couch"), ("sofa", "couch"),
                         ("dining room", "dining"), ("table", "dining"), ("workshop", "workshop door")):
        if alias in q:
            if canon in lower:
                return lower[canon]
            # "couch" -> first place whose name starts with it (couch right)
            for name in places:
                if name.lower().startswith(canon):
                    return name
    hit = _fuzzy_pick(q, places)
    if hit:
        return hit
    # share a word with a place name ("over to the couch" -> "couch right")
    words = set(re.findall(r"[a-z]+", q))
    best, best_n = None, 0
    for name in places:
        n = len(words & set(name.lower().split()))
        if n > best_n:
            best, best_n = name, n
    return best


__all__ = ["Brain", "Navigator", "NoNavigator", "TaskRunner", "MODEL", "_match_place"]
