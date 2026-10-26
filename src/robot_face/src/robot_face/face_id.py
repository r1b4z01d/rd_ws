"""Face identification for the robot face — learns to tell people apart.

Pipeline (all built into OpenCV ≥ 4.5.4, no extra packages):
  * YuNet  (``cv2.FaceDetectorYN``)   — finds faces + 5 landmarks in a BGR frame
  * SFace  (``cv2.FaceRecognizerSF``) — aligns a face and produces a 128-d embedding

A :class:`FaceGallery` keeps a few embeddings per person and is persisted as
JSON. Unknown faces that are seen consistently for a few frames are enrolled
automatically as ``person_N``; call :meth:`FaceIdentifier.name_person` to give
them a real name later.

Cosine-similarity threshold 0.363 is the value recommended in the OpenCV docs
for SFace; higher is a stricter match.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

DEFAULT_DETECTOR_MODEL = "face_detection_yunet_2023mar.onnx"
DEFAULT_RECOGNIZER_MODEL = "face_recognition_sface_2021dec.onnx"


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class Person:
    person_id: str
    name: Optional[str] = None
    embeddings: List[np.ndarray] = field(default_factory=list)
    created: float = field(default_factory=time.time)
    last_seen: float = 0.0
    seen_count: int = 0

    @property
    def label(self) -> str:
        return self.name or self.person_id


@dataclass
class Match:
    """Result of one identification."""
    person_id: Optional[str]      # None when the face is unknown and not (yet) enrolled
    name: Optional[str]
    score: float                  # cosine similarity to the best gallery entry (0 if none)
    is_new: bool                  # True on the frame a person was just enrolled
    box: Tuple[int, int, int, int]  # x, y, w, h of the face in the frame
    face_score: float             # detector confidence

    @property
    def label(self) -> Optional[str]:
        return self.name or self.person_id

    @property
    def known(self) -> bool:
        return self.person_id is not None


# ---------------------------------------------------------------------------
# Gallery (persistence)
# ---------------------------------------------------------------------------
class FaceGallery:
    def __init__(self, path: Optional[str], max_embeddings_per_person: int = 10):
        self.path = os.path.expanduser(path) if path else None
        self.max_per_person = max(1, int(max_embeddings_per_person))
        self.people: Dict[str, Person] = {}
        self._next_index = 1
        if self.path and os.path.exists(self.path):
            self.load()

    # -- persistence --------------------------------------------------------
    def load(self) -> None:
        with open(self.path, "r") as f:
            data = json.load(f)
        self.people = {}
        for pid, rec in data.get("people", {}).items():
            self.people[pid] = Person(
                person_id=pid,
                name=rec.get("name"),
                embeddings=[np.asarray(e, dtype=np.float32) for e in rec.get("embeddings", [])],
                created=float(rec.get("created", 0.0)),
                last_seen=float(rec.get("last_seen", 0.0)),
                seen_count=int(rec.get("seen_count", 0)),
            )
        self._next_index = int(data.get("next_index", len(self.people) + 1))

    def save(self) -> None:
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        data = {
            "next_index": self._next_index,
            "people": {
                pid: {
                    "name": p.name,
                    "embeddings": [e.astype(float).round(6).tolist() for e in p.embeddings],
                    "created": p.created,
                    "last_seen": p.last_seen,
                    "seen_count": p.seen_count,
                }
                for pid, p in self.people.items()
            },
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, self.path)

    # -- queries ------------------------------------------------------------
    def best_match(self, embedding: np.ndarray) -> Tuple[Optional[Person], float]:
        """Return (person, best cosine similarity) across the gallery."""
        best_p, best_s = None, -1.0
        for p in self.people.values():
            for e in p.embeddings:
                s = _cosine(embedding, e)
                if s > best_s:
                    best_p, best_s = p, s
        return best_p, (best_s if best_p is not None else 0.0)

    def find(self, id_or_name: str) -> Optional[Person]:
        if id_or_name in self.people:
            return self.people[id_or_name]
        low = id_or_name.strip().lower()
        for p in self.people.values():
            if p.name and p.name.lower() == low:
                return p
        return None

    # -- mutations ----------------------------------------------------------
    def enroll(self, embedding: np.ndarray, name: Optional[str] = None) -> Person:
        pid = f"person_{self._next_index}"
        self._next_index += 1
        p = Person(person_id=pid, name=name, embeddings=[embedding.astype(np.float32)])
        p.last_seen = time.time()
        p.seen_count = 1
        self.people[pid] = p
        self.save()
        return p

    def add_embedding(self, person: Person, embedding: np.ndarray) -> None:
        """Add a new view of a known person, keeping the most recent N."""
        person.embeddings.append(embedding.astype(np.float32))
        if len(person.embeddings) > self.max_per_person:
            # Keep the first (anchor) embedding and the most recent ones
            person.embeddings = [person.embeddings[0]] + person.embeddings[-(self.max_per_person - 1):]
        self.save()

    def merge(self, source: Person, target: Person) -> Person:
        """Fold ``source`` into ``target`` (same real person enrolled twice)."""
        for e in source.embeddings:
            if all(_cosine(e, d) < 0.92 for d in target.embeddings):
                target.embeddings.append(e)
        if len(target.embeddings) > self.max_per_person:
            target.embeddings = [target.embeddings[0]] + target.embeddings[-(self.max_per_person - 1):]
        target.seen_count += source.seen_count
        target.last_seen = max(target.last_seen, source.last_seen)
        target.created = min(target.created, source.created)
        del self.people[source.person_id]
        self.save()
        return target

    def rename(self, id_or_name: str, new_name: str) -> Optional[Person]:
        """Name a person. If another person already has that name, the two are
        merged (they were the same person enrolled twice) and the named one is
        returned."""
        p = self.find(id_or_name)
        if p is None:
            return None
        new_name = new_name.strip()
        existing = self.find(new_name) if new_name else None
        if existing is not None and existing is not p:
            return self.merge(p, existing)
        p.name = new_name or None
        self.save()
        return p

    def forget(self, id_or_name: str) -> bool:
        p = self.find(id_or_name)
        if p is None:
            return False
        del self.people[p.person_id]
        self.save()
        return True

    def summary(self) -> List[str]:
        return [
            f"{p.person_id}: {p.name or '(unnamed)'} — {len(p.embeddings)} views, seen {p.seen_count}x"
            for p in sorted(self.people.values(), key=lambda q: q.created)
        ]


def _cosine(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0.0 or nb == 0.0:
        return 0.0
    return float(np.dot(a, b) / (na * nb))


# ---------------------------------------------------------------------------
# Identifier
# ---------------------------------------------------------------------------
class FaceIdentifier:
    """Detect → embed → match/enroll, with a little temporal hysteresis.

    ``identify()`` is meant to be called every few camera frames. It returns a
    :class:`Match` for the chosen face, or ``None`` if no usable face was found.
    """

    def __init__(
        self,
        models_dir: str,
        gallery_path: Optional[str],
        match_threshold: float = 0.363,
        auto_enroll: bool = True,
        min_face_px: int = 40,
        min_detect_score: float = 0.85,
        enroll_frames: int = 6,
        learn_threshold: float = 0.55,
        max_embeddings_per_person: int = 10,
        unknown_threshold: float = 0.28,
        enroll_min_face_px: int = 64,
        sticky_seconds: float = 2.5,
        detector_model: str = DEFAULT_DETECTOR_MODEL,
        recognizer_model: str = DEFAULT_RECOGNIZER_MODEL,
    ):
        det_path = os.path.join(models_dir, detector_model)
        rec_path = os.path.join(models_dir, recognizer_model)
        for pth in (det_path, rec_path):
            if not os.path.exists(pth):
                raise FileNotFoundError(f"face model not found: {pth}")

        self._detector = cv2.FaceDetectorYN.create(det_path, "", (320, 320), 0.6, 0.3, 5000)
        self._recognizer = cv2.FaceRecognizerSF.create(rec_path, "")
        self._input_size: Tuple[int, int] = (0, 0)

        self.gallery = FaceGallery(gallery_path, max_embeddings_per_person)
        self.match_threshold = float(match_threshold)
        self.learn_threshold = float(learn_threshold)
        self.auto_enroll = bool(auto_enroll)
        self.min_face_px = int(min_face_px)
        self.min_detect_score = float(min_detect_score)
        self.enroll_frames = max(1, int(enroll_frames))
        # Scores between unknown_threshold and match_threshold are "not sure":
        # neither a match nor grounds to enroll a new person.
        self.unknown_threshold = min(float(unknown_threshold), self.match_threshold)
        self.enroll_min_face_px = max(int(enroll_min_face_px), self.min_face_px)
        self.sticky_seconds = float(sticky_seconds)

        # Consecutive sightings of the same unknown face, awaiting enrollment
        self._pending: List[np.ndarray] = []
        # Most recent confident match: (person_id, time). Keeps the identity
        # through brief bad frames (turned head, distance) instead of re-enrolling.
        self._last_known: Optional[Tuple[str, float]] = None

    # -- detection ----------------------------------------------------------
    def detect(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Run YuNet; returns an (N, 15) array [x, y, w, h, 5×(lx, ly), score]."""
        h, w = frame_bgr.shape[:2]
        if (w, h) != self._input_size:
            self._detector.setInputSize((w, h))
            self._input_size = (w, h)
        _, faces = self._detector.detect(frame_bgr)
        if faces is None:
            return np.zeros((0, 15), dtype=np.float32)
        return faces

    def embed(self, frame_bgr: np.ndarray, face_row: np.ndarray) -> np.ndarray:
        aligned = self._recognizer.alignCrop(frame_bgr, face_row)
        feat = self._recognizer.feature(aligned)
        return np.asarray(feat, dtype=np.float32).ravel()

    # -- main entry ---------------------------------------------------------
    def identify(
        self,
        frame_bgr: np.ndarray,
        near: Optional[Tuple[float, float]] = None,
    ) -> Optional[Match]:
        """Identify the most relevant face in ``frame_bgr``.

        ``near`` is an optional (x, y) pixel hint — e.g. the tracked person's head
        position — used to pick the right face when several are visible.
        Otherwise the largest face wins.
        """
        faces = self.detect(frame_bgr)
        if faces.shape[0] == 0:
            self._pending.clear()
            return None

        good = faces[(faces[:, 14] >= self.min_detect_score) & (faces[:, 2] >= self.min_face_px)]
        if good.shape[0] == 0:
            return None

        if near is not None:
            cx = good[:, 0] + good[:, 2] / 2.0
            cy = good[:, 1] + good[:, 3] / 2.0
            idx = int(np.argmin((cx - near[0]) ** 2 + (cy - near[1]) ** 2))
        else:
            idx = int(np.argmax(good[:, 2] * good[:, 3]))
        row = good[idx]
        box = tuple(int(v) for v in row[:4])
        face_w = float(row[2])
        face_score = float(row[14])

        emb = self.embed(frame_bgr, row)
        person, score = self.gallery.best_match(emb)
        now = time.time()

        if person is not None and score >= self.match_threshold:
            self._pending.clear()
            self._last_known = (person.person_id, now)
            person.last_seen = now
            person.seen_count += 1
            # Learn new views of a confidently matched person (clear, frontal-ish faces)
            if score >= self.learn_threshold and len(person.embeddings) < self.gallery.max_per_person:
                if all(_cosine(emb, e) < 0.92 for e in person.embeddings):   # only genuinely new views
                    self.gallery.add_embedding(person, emb)
            return Match(person.person_id, person.name, score, False, box, face_score)

        sticky = (
            self._last_known is not None
            and (now - self._last_known[1]) <= self.sticky_seconds
            and self._last_known[0] in self.gallery.people
        )

        # Not sure: looks a bit like someone we know, but not enough to be certain.
        if person is not None and score >= self.unknown_threshold:
            self._pending.clear()
            if sticky and self._last_known[0] == person.person_id:
                # Same person we were just confidently seeing — keep the identity and
                # learn this harder view so it matches directly next time.
                if len(person.embeddings) < self.gallery.max_per_person:
                    self.gallery.add_embedding(person, emb)
                person.last_seen = now
                return Match(person.person_id, person.name, score, False, box, face_score)
            return Match(None, None, score, False, box, face_score)

        # Clearly nobody we know. If we were confidently seeing someone a moment ago,
        # assume a bad frame (turned head) rather than a new person.
        if sticky:
            known = self.gallery.people[self._last_known[0]]
            return Match(known.person_id, known.name, score, False, box, face_score)

        if not self.auto_enroll or face_w < self.enroll_min_face_px:
            # Too far away to learn reliably; wait until they come closer
            return Match(None, None, score, False, box, face_score)

        # Require a run of consistent sightings before enrolling, to avoid
        # learning blurry glimpses or mis-detections as new people.
        if self._pending and _cosine(emb, self._pending[-1]) < self.match_threshold:
            self._pending.clear()
        self._pending.append(emb)
        if len(self._pending) < self.enroll_frames:
            return Match(None, None, score, False, box, face_score)

        anchor = np.mean(np.stack(self._pending), axis=0).astype(np.float32)
        new_person = self.gallery.enroll(anchor)
        for e in self._pending[-3:]:
            if _cosine(e, anchor) < 0.98:
                self.gallery.add_embedding(new_person, e)
        self._pending.clear()
        self._last_known = (new_person.person_id, now)
        return Match(new_person.person_id, None, 1.0, True, box, face_score)

    # -- management ---------------------------------------------------------
    def name_person(self, id_or_name: str, new_name: str) -> Optional[Person]:
        return self.gallery.rename(id_or_name, new_name)

    def forget(self, id_or_name: str) -> bool:
        self._last_known = None
        return self.gallery.forget(id_or_name)

    def people(self) -> List[Person]:
        return list(self.gallery.people.values())


__all__ = ["FaceIdentifier", "FaceGallery", "Match", "Person",
           "DEFAULT_DETECTOR_MODEL", "DEFAULT_RECOGNIZER_MODEL"]
