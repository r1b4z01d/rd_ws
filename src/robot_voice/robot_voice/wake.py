"""Wake-phrase detection on transcripts ("Robot Disco, turn on the light").

Whisper spells the name in a few ways (robot disco / robo disco / robot
discos / robodisco), so matching is fuzzy on the first few words. The wake
phrase may also stand alone ("Robot Disco?"), in which case the caller opens a
short follow-up window where the next utterance needs no wake word.
"""

from __future__ import annotations

import difflib
import re
from dataclasses import dataclass
from typing import List, Optional, Sequence

_PUNCT = re.compile(r"[^\w\s']+")
_LEAD = ("hey", "ok", "okay", "yo", "hi", "hello", "um", "uh", "so")


@dataclass
class WakeResult:
    woke: bool
    command: str          # text after the wake phrase ("" if the wake word stood alone)
    matched: str          # the words that matched the wake phrase
    score: float


def _norm(text: str) -> List[str]:
    text = _PUNCT.sub(" ", text.lower().replace("-", " "))
    return [t for t in text.split() if t]


class WakeMatcher:
    def __init__(self, phrases: Sequence[str] = ("robot disco",), threshold: float = 0.74,
                 max_lead_words: int = 2):
        self.phrases = [" ".join(_norm(p)) for p in phrases if p.strip()] or ["robot disco"]
        self.threshold = float(threshold)
        self.max_lead = int(max_lead_words)

    @staticmethod
    def _parts_ok(cand_words: List[str], phrase: str) -> bool:
        """Both halves of the phrase must be present: the candidate must end in
        something like the last word ("disco") and start with something like the
        rest ("robot"). Stops "Rob Lowe's disco" from waking the robot while
        still accepting "robo disco", "robodisco" and "robot discos"."""
        pw = phrase.split()
        if len(pw) < 2:
            return True
        last, head_ph = pw[-1], "".join(pw[:-1])
        j = "".join(cand_words)
        if j.endswith(last + "'s") or j.endswith(last + "s"):
            j = j[: j.rfind(last) + len(last)]
        tail, head = j[-len(last):], j[: -len(last)]
        if difflib.SequenceMatcher(None, tail, last).ratio() < 0.8:
            return False
        return difflib.SequenceMatcher(None, head, head_ph).ratio() >= 0.75

    def _best(self, words: List[str]) -> tuple:
        """Best (score, n_words_consumed, phrase) for a wake phrase at the start of ``words``."""
        best = (0.0, 0, "")
        for phrase in self.phrases:
            n = len(phrase.split())
            for take in (n - 1, n, n + 1):          # "robodisco" (1 word) .. "robot dis co" (3)
                if take <= 0 or take > len(words):
                    continue
                cand = words[:take]
                if not self._parts_ok(cand, phrase):
                    continue
                score = difflib.SequenceMatcher(None, "".join(cand), phrase.replace(" ", "")).ratio()
                if score > best[0]:
                    best = (score, take, phrase)
        return best

    def match(self, text: str) -> WakeResult:
        words = _norm(text)
        if not words:
            return WakeResult(False, "", "", 0.0)
        # allow a couple of lead-in words ("hey robot disco")
        for skip in range(0, min(self.max_lead, len(words) - 1) + 1):
            if skip and words[skip - 1] not in _LEAD:
                break
            score, take, phrase = self._best(words[skip:])
            if score >= self.threshold:
                matched = " ".join(words[skip:skip + take])
                rest = words[skip + take:]
                # drop a dangling comma-word like "please" at the very start? keep it; strip "can you"
                return WakeResult(True, " ".join(rest), matched, score)
        return WakeResult(False, "", "", 0.0)


__all__ = ["WakeMatcher", "WakeResult"]
