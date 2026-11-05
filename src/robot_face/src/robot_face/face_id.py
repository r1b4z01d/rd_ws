"""Face identification for the robot face — learns to tell people apart.

Two interchangeable back-ends produce faces + embeddings:

* :class:`OpenCVFaceBackend` — YuNet detector + SFace 128-d embedding, built
  into OpenCV ≥ 4.5.4; small and CPU-cheap, weaker on off-angle/distant faces.
* :class:`InsightFaceBackend` — SCRFD detector + ArcFace 512-d embedding
  (``buffalo_l``) via ``faceid_worker.py`` running in its own virtualenv;
  markedly more robust. Preferred when its venv exists.

A :class:`FaceGallery` keeps a few embeddings per person and is persisted as
JSON. Unknown faces that are seen consistently for a few frames are enrolled
automatically as ``person_N``; call :meth:`FaceIdentifier.name_person` to give
them a real name later. Naming someone with a name that already exists merges
the two records.
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

DEFAULT_DETECTOR_MODEL = "face_detection_yunet_2023mar.onnx"
DEFAULT_RECOGNIZER_MODEL = "face_recognition_sface_2021dec.onnx"
WORKER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "faceid_worker.py")


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
    thumb_width: int = 0          # face width (px) of the saved thumbnail, 0 = none
    thumb_time: float = 0.0
    last_greeted: float = 0.0     # epoch of the last spoken hello (greeting cooldown; survives restarts)

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


@dataclass
class FaceObs:
    """One detected face, as produced by a back-end."""
    box: Tuple[int, int, int, int]            # x, y, w, h
    score: float                              # detector confidence
    embedding: Optional[np.ndarray] = None    # filled eagerly by InsightFace, lazily by OpenCV
    raw: Optional[np.ndarray] = None          # back-end specific (YuNet row)


# ---------------------------------------------------------------------------
# Gallery (persistence)
# ---------------------------------------------------------------------------
class FaceGallery:
    def __init__(self, path: Optional[str], max_embeddings_per_person: int = 10,
                 expected_dim: Optional[int] = None, backend: str = "",
                 thumb_dir: Optional[str] = None):
        self.path = os.path.expanduser(path) if path else None
        self.max_per_person = max(1, int(max_embeddings_per_person))
        self.expected_dim = expected_dim
        self.backend = backend
        self.thumb_dir = os.path.expanduser(thumb_dir) if thumb_dir else (
            os.path.join(os.path.dirname(self.path), "faces") if self.path else None)
        self.people: Dict[str, Person] = {}
        self._next_index = 1
        self.dropped_on_load = 0
        self.lock = threading.RLock()     # the vision thread and the web UI both touch the gallery
        if self.path and os.path.exists(self.path):
            self.load()

    # -- thumbnails ---------------------------------------------------------
    def thumb_path(self, person_id: str) -> Optional[str]:
        if not self.thumb_dir:
            return None
        return os.path.join(self.thumb_dir, f"{person_id}.jpg")

    def save_thumb(self, person: Person, frame_bgr: np.ndarray, box: Tuple[int, int, int, int],
                   force: bool = False) -> bool:
        """Keep one face picture per person: the first one, then a bigger (closer) face,
        or a fresher one every few hours."""
        if not self.thumb_dir:
            return False
        x, y, w, h = box
        better = w >= person.thumb_width * 1.2
        stale = time.time() - person.thumb_time > 6 * 3600
        if not (force or person.thumb_width == 0 or better or stale):
            return False
        H, W = frame_bgr.shape[:2]
        cx, cy, side = x + w / 2.0, y + h / 2.0, max(w, h) * 1.6
        x0, y0 = int(max(0, cx - side / 2)), int(max(0, cy - side / 2))
        x1, y1 = int(min(W, cx + side / 2)), int(min(H, cy + side / 2))
        if x1 - x0 < 24 or y1 - y0 < 24:
            return False
        crop = cv2.resize(frame_bgr[y0:y1, x0:x1], (192, 192), interpolation=cv2.INTER_AREA)
        os.makedirs(self.thumb_dir, exist_ok=True)
        tmp = self.thumb_path(person.person_id) + ".tmp.jpg"
        if not cv2.imwrite(tmp, crop, [cv2.IMWRITE_JPEG_QUALITY, 88]):
            return False
        os.replace(tmp, self.thumb_path(person.person_id))
        person.thumb_width, person.thumb_time = int(w), time.time()
        return True

    # -- persistence --------------------------------------------------------
    def load(self) -> None:
        with open(self.path, "r") as f:
            data = json.load(f)
        self.people = {}
        self.dropped_on_load = 0
        for pid, rec in data.get("people", {}).items():
            embs = [np.asarray(e, dtype=np.float32) for e in rec.get("embeddings", [])]
            if self.expected_dim is not None:
                kept = [e for e in embs if e.size == self.expected_dim]
                self.dropped_on_load += len(embs) - len(kept)
                embs = kept
                if not embs:
                    continue    # a record from another embedding space is useless here
            self.people[pid] = Person(
                person_id=pid,
                name=rec.get("name"),
                embeddings=embs,
                created=float(rec.get("created", 0.0)),
                last_seen=float(rec.get("last_seen", 0.0)),
                seen_count=int(rec.get("seen_count", 0)),
                thumb_width=int(rec.get("thumb_width", 0)),
                thumb_time=float(rec.get("thumb_time", 0.0)),
                last_greeted=float(rec.get("last_greeted", 0.0)),
            )
        self._next_index = int(data.get("next_index", len(self.people) + 1))

    def save(self) -> None:
        if not self.path:
            return
        with self.lock:
            self._save_locked()

    def _save_locked(self) -> None:
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        data = {
            "backend": self.backend,
            "dim": self.expected_dim,
            "next_index": self._next_index,
            "people": {
                pid: {
                    "name": p.name,
                    "embeddings": [e.astype(float).round(6).tolist() for e in p.embeddings],
                    "created": p.created,
                    "last_seen": p.last_seen,
                    "seen_count": p.seen_count,
                    "thumb_width": p.thumb_width,
                    "thumb_time": p.thumb_time,
                    "last_greeted": p.last_greeted,
                }
                for pid, p in self.people.items()
            },
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, self.path)

    # -- greeting cooldown ----------------------------------------------------
    def mark_greeted(self, person_id: str, when: Optional[float] = None) -> None:
        with self.lock:
            p = self.people.get(person_id)
            if p is None:
                return
            p.last_greeted = float(when if when is not None else time.time())
            self._save_locked()

    def reset_greeting(self, person_id: str) -> bool:
        """Forget when this person was last greeted, so the next sighting gets a hello."""
        with self.lock:
            p = self.people.get(person_id)
            if p is None:
                return False
            p.last_greeted = 0.0
            self._save_locked()
            return True

    # -- queries ------------------------------------------------------------
    def best_match(self, embedding: np.ndarray) -> Tuple[Optional[Person], float]:
        """Return (person, best cosine similarity) across the gallery."""
        best_p, best_s = None, -1.0
        with self.lock:
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
        with self.lock:
            pid = f"person_{self._next_index}"
            self._next_index += 1
            p = Person(person_id=pid, name=name, embeddings=[embedding.astype(np.float32)])
            p.last_seen = time.time()
            p.seen_count = 1
            self.people[pid] = p
            self._save_locked()
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
        with self.lock:
            return self._merge_locked(source, target)

    def _merge_locked(self, source: Person, target: Person) -> Person:
        for e in source.embeddings:
            if all(_cosine(e, d) < 0.92 for d in target.embeddings):
                target.embeddings.append(e)
        if len(target.embeddings) > self.max_per_person:
            target.embeddings = [target.embeddings[0]] + target.embeddings[-(self.max_per_person - 1):]
        target.seen_count += source.seen_count
        target.last_seen = max(target.last_seen, source.last_seen)
        target.created = min(target.created, source.created)
        if target.thumb_width == 0 and source.thumb_width and self.thumb_dir:
            try:
                os.replace(self.thumb_path(source.person_id), self.thumb_path(target.person_id))
                target.thumb_width, target.thumb_time = source.thumb_width, source.thumb_time
            except OSError:
                pass
        elif self.thumb_dir:
            try:
                os.remove(self.thumb_path(source.person_id))
            except OSError:
                pass
        del self.people[source.person_id]
        self._save_locked()
        return target

    def rename(self, id_or_name: str, new_name: str) -> Optional[Person]:
        """Name a person. If another person already has that name, the two are
        merged (they were the same person enrolled twice) and the named one is
        returned."""
        with self.lock:
            p = self.find(id_or_name)
            if p is None:
                return None
            new_name = new_name.strip()
            existing = self.find(new_name) if new_name else None
            if existing is not None and existing is not p:
                return self._merge_locked(p, existing)
            p.name = new_name or None
            self._save_locked()
            return p

    def forget(self, id_or_name: str) -> bool:
        with self.lock:
            p = self.find(id_or_name)
            if p is None:
                return False
            del self.people[p.person_id]
            if self.thumb_dir:
                try:
                    os.remove(self.thumb_path(p.person_id))
                except OSError:
                    pass
            self._save_locked()
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
# Back-ends
# ---------------------------------------------------------------------------
class OpenCVFaceBackend:
    """YuNet + SFace through OpenCV's own ``objdetect`` module."""

    name = "opencv"
    dim = 128
    # cosine-similarity defaults (OpenCV docs recommend 0.363 for SFace)
    defaults = {"match": 0.363, "unknown": 0.28, "learn": 0.55}

    def __init__(self, models_dir: str,
                 detector_model: str = DEFAULT_DETECTOR_MODEL,
                 recognizer_model: str = DEFAULT_RECOGNIZER_MODEL):
        det_path = os.path.join(models_dir, detector_model)
        rec_path = os.path.join(models_dir, recognizer_model)
        for pth in (det_path, rec_path):
            if not os.path.exists(pth):
                raise FileNotFoundError(f"face model not found: {pth}")
        self._detector = cv2.FaceDetectorYN.create(det_path, "", (320, 320), 0.6, 0.3, 5000)
        self._recognizer = cv2.FaceRecognizerSF.create(rec_path, "")
        self._input_size: Tuple[int, int] = (0, 0)
        self.info = "YuNet + SFace (OpenCV, CPU)"

    def analyze(self, frame_bgr: np.ndarray, embed: bool = True) -> List[FaceObs]:
        h, w = frame_bgr.shape[:2]
        if (w, h) != self._input_size:
            self._detector.setInputSize((w, h))
            self._input_size = (w, h)
        _, faces = self._detector.detect(frame_bgr)
        if faces is None:
            return []
        out = []
        for row in faces:
            out.append(FaceObs(
                box=tuple(int(v) for v in row[:4]),
                score=float(row[14]),
                raw=row,
            ))
        return out

    def embed(self, frame_bgr: np.ndarray, obs: FaceObs) -> np.ndarray:
        aligned = self._recognizer.alignCrop(frame_bgr, obs.raw)
        feat = self._recognizer.feature(aligned)
        return np.asarray(feat, dtype=np.float32).ravel()

    def stop(self) -> None:
        pass


class InsightFaceBackend:
    """SCRFD + ArcFace via ``faceid_worker.py`` in its own venv (see the worker's
    docstring for the pipe protocol). Thread-safe; one request at a time."""

    name = "arcface"
    dim = 512
    # ArcFace (w600k_r50) cosine similarities: same person typically > 0.5,
    # different people < 0.3
    defaults = {"match": 0.45, "unknown": 0.30, "learn": 0.60}

    def __init__(self, python: str = "~/.robot_face/faceid-venv/bin/python",
                 model: str = "buffalo_l", det_size: int = 480, gpu: bool = False,
                 max_faces: int = 3, threads: int = 2, start_timeout: float = 120.0,
                 request_timeout: float = 10.0, log=None):
        self.python = os.path.expanduser(python)
        self.model = model
        self.det_size = int(det_size)
        self.gpu = bool(gpu)
        self.max_faces = int(max_faces)
        self.threads = max(1, int(threads))
        self.start_timeout = float(start_timeout)
        self.request_timeout = float(request_timeout)
        self._log = log or (lambda msg: None)
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._restarts = 0
        self.providers: List[str] = []
        self.info = f"SCRFD + ArcFace ({model})"
        self.last_ms = 0.0

    @staticmethod
    def available(python: str = "~/.robot_face/faceid-venv/bin/python") -> bool:
        return os.path.exists(os.path.expanduser(python)) and os.path.exists(WORKER_SCRIPT)

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> bool:
        with self._lock:
            return self._start_locked()

    def _start_locked(self) -> bool:
        self._stop_locked()
        args = [self.python, WORKER_SCRIPT, "--model", self.model,
                "--det-size", str(self.det_size), "--max-faces", str(self.max_faces),
                "--threads", str(self.threads)]
        if self.gpu:
            args.append("--gpu")
        try:
            self._proc = subprocess.Popen(
                args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            )
        except OSError as e:
            self._log(f"face worker failed to launch: {e}")
            self._proc = None
            return False
        ready = self._read_reply(self.start_timeout)
        if not ready or not ready.get("ready"):
            self._log(f"face worker did not become ready: {ready}")
            self._stop_locked()
            return False
        self.providers = list(ready.get("providers", []))
        self.info = f"SCRFD + ArcFace ({self.model}, {self.providers[0] if self.providers else '?'}, det {self.det_size})"
        return True

    def _read_reply(self, timeout: float) -> Optional[dict]:
        if self._proc is None or self._proc.stdout is None:
            return None
        result: List[Optional[bytes]] = [None]

        def _reader():
            try:
                result[0] = self._proc.stdout.readline()
            except Exception:
                result[0] = None

        th = threading.Thread(target=_reader, daemon=True)
        th.start()
        th.join(timeout)
        line = result[0]
        if th.is_alive() or not line:
            return None
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return None

    def _stop_locked(self) -> None:
        p = self._proc
        self._proc = None
        if p is None:
            return
        try:
            if p.stdin:
                p.stdin.write(b'{"cmd": "quit"}\n')
                p.stdin.flush()
            p.wait(timeout=2.0)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    # -- analysis -----------------------------------------------------------
    def analyze(self, frame_bgr: np.ndarray, embed: bool = True) -> List[FaceObs]:
        frame = np.ascontiguousarray(frame_bgr)
        if frame.ndim != 3 or frame.dtype != np.uint8:
            raise ValueError("expected an HxWx3 uint8 BGR frame")
        h, w, c = frame.shape
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                if self._restarts >= 3 or not self._start_locked():
                    return []
                self._restarts += 1
            header = json.dumps({"w": w, "h": h, "c": c, "max_faces": self.max_faces, "embed": bool(embed)}) + "\n"
            try:
                self._proc.stdin.write(header.encode("ascii"))
                self._proc.stdin.write(frame.tobytes())
                self._proc.stdin.flush()
            except (BrokenPipeError, OSError, AttributeError) as e:
                self._log(f"face worker pipe broken: {e}")
                self._stop_locked()
                return []
            reply = self._read_reply(self.request_timeout)
        if reply is None:
            self._log("face worker timed out; restarting it")
            self.stop()
            return []
        if not reply.get("ok"):
            self._log(f"face worker error: {reply.get('error')}")
            return []
        self.last_ms = float(reply.get("ms", 0.0))
        out = []
        for f in reply.get("faces", []):
            out.append(FaceObs(
                box=tuple(int(v) for v in f["box"]),
                score=float(f["score"]),
                embedding=np.asarray(f["emb"], dtype=np.float32) if "emb" in f else None,
            ))
        return out

    def embed(self, frame_bgr: np.ndarray, obs: FaceObs) -> np.ndarray:
        if obs.embedding is not None:
            return obs.embedding
        # Shouldn't happen (the worker embeds eagerly) — re-run and take the nearest box
        faces = self.analyze(frame_bgr)
        if not faces:
            raise RuntimeError("no face to embed")
        bx, by = obs.box[0] + obs.box[2] / 2.0, obs.box[1] + obs.box[3] / 2.0
        best = min(faces, key=lambda f: (f.box[0] + f.box[2] / 2.0 - bx) ** 2 + (f.box[1] + f.box[3] / 2.0 - by) ** 2)
        return best.embedding


# ---------------------------------------------------------------------------
# Identifier
# ---------------------------------------------------------------------------
class FaceIdentifier:
    """Detect → embed → match/enroll, with a little temporal hysteresis.

    ``identify()`` is meant to be called every few camera frames (from a worker
    thread when the back-end is slow). It returns a :class:`Match` for the
    chosen face, or ``None`` if no usable face was found.
    """

    def __init__(
        self,
        backend,
        gallery_path: Optional[str],
        match_threshold: Optional[float] = None,   # None → back-end default
        auto_enroll: bool = True,
        min_face_px: int = 40,
        min_detect_score: float = 0.6,
        enroll_frames: int = 6,
        learn_threshold: Optional[float] = None,
        max_embeddings_per_person: int = 10,
        unknown_threshold: Optional[float] = None,
        enroll_min_face_px: int = 64,
        sticky_seconds: float = 2.5,
        thumb_dir: Optional[str] = None,
    ):
        self.backend = backend
        d = backend.defaults
        self.match_threshold = float(match_threshold if match_threshold else d["match"])
        self.learn_threshold = float(learn_threshold if learn_threshold else d["learn"])
        unk = float(unknown_threshold if unknown_threshold else d["unknown"])
        # Scores between unknown_threshold and match_threshold are "not sure":
        # neither a match nor grounds to enroll a new person.
        self.unknown_threshold = min(unk, self.match_threshold)
        self.gallery = FaceGallery(gallery_path, max_embeddings_per_person,
                                   expected_dim=backend.dim, backend=backend.name, thumb_dir=thumb_dir)
        self.auto_enroll = bool(auto_enroll)
        self.min_face_px = int(min_face_px)
        self.min_detect_score = float(min_detect_score)
        self.enroll_frames = max(1, int(enroll_frames))
        self.enroll_min_face_px = max(int(enroll_min_face_px), self.min_face_px)
        self.sticky_seconds = float(sticky_seconds)

        # Consecutive sightings of the same unknown face, awaiting enrollment
        self._pending: List[np.ndarray] = []
        # Most recent confident match: (person_id, time). Keeps the identity
        # through brief bad frames (turned head, distance) instead of re-enrolling.
        self._last_known: Optional[Tuple[str, float]] = None
        # Folks page asked for a fresh picture of this person on their next match
        self._snap_request: Optional[str] = None

    # -- convenience constructor -------------------------------------------
    @classmethod
    def create(cls, backend_name: str, opencv_models_dir: str, gallery_path: Optional[str],
               faceid_python: str = "~/.robot_face/faceid-venv/bin/python",
               arcface_model: str = "buffalo_l", det_size: int = 480, gpu: bool = False,
               threads: int = 2, log=None, **kwargs) -> "FaceIdentifier":
        """Build an identifier for ``backend_name`` (auto | arcface | opencv).
        ``auto`` prefers ArcFace when its venv exists. The gallery file gets a
        per-back-end suffix (people.json → people.arcface.json) because the two
        embedding spaces are incompatible."""
        name = (backend_name or "auto").lower()
        backend = None
        if name in ("auto", "arcface", "insightface"):
            if InsightFaceBackend.available(faceid_python):
                b = InsightFaceBackend(python=faceid_python, model=arcface_model,
                                       det_size=det_size, gpu=gpu, threads=threads, log=log)
                if b.start():
                    backend = b
                elif name != "auto":
                    raise RuntimeError("ArcFace worker failed to start")
            elif name != "auto":
                raise FileNotFoundError(f"ArcFace venv not found at {faceid_python}")
        if backend is None:
            backend = OpenCVFaceBackend(opencv_models_dir)
        path = gallery_path
        if path and backend.name != "opencv":
            root, ext = os.path.splitext(os.path.expanduser(path))
            path = f"{root}.{backend.name}{ext or '.json'}"
        return cls(backend, path, **kwargs)

    # -- main entry ---------------------------------------------------------
    def identify(
        self,
        frame_bgr: np.ndarray,
        near: Optional[Tuple[float, float]] = None,
        faces: Optional[List[FaceObs]] = None,
    ) -> Optional[Match]:
        """Identify the most relevant face in ``frame_bgr``.

        ``near`` is an optional (x, y) pixel hint — e.g. the tracked person's head
        position — used to pick the right face when several are visible.
        Otherwise the largest face wins. ``faces`` may carry the detections of a
        pass the caller already ran (with embeddings) to avoid detecting twice.
        """
        if faces is None:
            faces = self.backend.analyze(frame_bgr, embed=True)
        if not faces:
            self._pending.clear()
            return None

        good = [f for f in faces if f.score >= self.min_detect_score and f.box[2] >= self.min_face_px]
        if not good:
            return None

        if near is not None:
            obs = min(good, key=lambda f: (f.box[0] + f.box[2] / 2.0 - near[0]) ** 2
                                        + (f.box[1] + f.box[3] / 2.0 - near[1]) ** 2)
        else:
            obs = max(good, key=lambda f: f.box[2] * f.box[3])
        box = obs.box
        face_w = float(box[2])
        face_score = obs.score

        emb = self.backend.embed(frame_bgr, obs)
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
            # Any confirmed match is good enough for a picture; a closer face replaces it later.
            force = self._snap_request == person.person_id
            if self.gallery.save_thumb(person, frame_bgr, box, force=force):
                self.gallery.save()
            if force:
                self._snap_request = None
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
        if self.gallery.save_thumb(new_person, frame_bgr, box, force=True):
            self.gallery.save()
        self._pending.clear()
        self._last_known = (new_person.person_id, now)
        return Match(new_person.person_id, None, 1.0, True, box, face_score)

    # -- management ---------------------------------------------------------
    def name_person(self, id_or_name: str, new_name: str) -> Optional[Person]:
        return self.gallery.rename(id_or_name, new_name)

    def forget(self, id_or_name: str) -> bool:
        self._last_known = None
        return self.gallery.forget(id_or_name)

    def request_snapshot(self, person_id: str) -> None:
        """Replace ``person_id``'s picture with the next confirmed sighting."""
        self._snap_request = person_id

    def people(self) -> List[Person]:
        return list(self.gallery.people.values())

    def stop(self) -> None:
        self.backend.stop()


__all__ = ["FaceIdentifier", "FaceGallery", "Match", "Person", "FaceObs",
           "OpenCVFaceBackend", "InsightFaceBackend",
           "DEFAULT_DETECTOR_MODEL", "DEFAULT_RECOGNIZER_MODEL"]
