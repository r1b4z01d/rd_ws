"""Landmark tasks: a policy run (an eval) tied to the place it is done at.

"Robot Disco, pick up tigger" → the landmarks file (kept by rd_ws/Scripts/route_editor.py, edited from the
manager's Landmarks tab) says that "pick up tigger" belongs to the Right Bookshelf and runs the model
tigger_1_merged_smolvla_1 with the instruction "Grab and hold tigger". The runner drives there (Nav2, blocking),
says so, runs VLA/scripts/vla_action.py (preflight + serve the model + eval_hw --auto) and reports how it ended.
One task at a time; "stop" cancels the drive or ends the run cleanly (SIGTERM = the eval's EXIT key: hand
open, arm home, UR powered down).

    landmarks.yaml
      landmarks:
        - id: right-bookshelf
          name: Right Bookshelf
          x: 3.1   y: -0.4   yaw: 1.57   map: zone_one
          tasks:
            - id: pick-up-tigger
              name: pick up tigger              # what you say
              model: tigger_1_merged_smolvla_1  # training run (or run@step) to serve
              task: Grab and hold tigger        # the instruction the policy was trained with
              seconds: 60  still: 3  done_when: closed  keep_grasp: true  home: true

ROS-free on purpose (the node passes in `navigate`, `say` and `log`), so the matching and the command line
are testable anywhere.
"""
from __future__ import annotations

import difflib
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from typing import Callable, List, Optional, Tuple

import yaml

# polite / filler words around the job's name: "could you please pick up tigger for me"
_PREFIX = re.compile(r"^(?:(?:hey|ok|okay|please|so|now|then|and)[, ]+)*"
                     r"(?:(?:can|could|would|will)\s+you\s+(?:please\s+)?|please\s+|go\s+(?:and\s+)?|go\s+ahead\s+and\s+|i\s+want\s+you\s+to\s+|i\s+need\s+you\s+to\s+)?")
# a question about a job is not an order to do it ("did you pick up tigger?", "how do you pick up tigger")
_QUESTION = re.compile(r"^(?:(?:hey|ok|okay|so|um|uh)[, ]+)*(?:did|have|had|do|does|are|were|was|is|what|where|when|why|how|who|which)\b")
_SUFFIX = re.compile(r"(?:\s+(?:please|for\s+me|now|right\s+now|again|would\s+you|will\s+you|ok|okay))*[.!?,\s]*$")


def normalise(text: str) -> str:
    t = re.sub(r"\s+", " ", (text or "").lower().strip())
    t = _PREFIX.sub("", t)
    t = _SUFFIX.sub("", t)
    return t.strip()


class LandmarkStore:
    """The landmarks YAML, re-read when its mtime changes (the editor rewrites the file atomically)."""

    def __init__(self, path: str):
        self.path = os.path.expanduser(path)
        self._mtime: Optional[float] = None
        self._items: List[dict] = []
        self._lock = threading.Lock()

    def landmarks(self) -> List[dict]:
        with self._lock:
            try:
                m = os.stat(self.path).st_mtime
            except OSError:
                self._items, self._mtime = [], None
                return []
            if m != self._mtime:
                try:
                    with open(self.path) as f:
                        data = yaml.safe_load(f) or {}
                    self._items = [i for i in (data.get("landmarks") or []) if isinstance(i, dict) and i.get("id")]
                    self._mtime = m
                except Exception:  # noqa: BLE001 — half-written file: keep the last good list
                    pass
            return list(self._items)

    def tasks(self) -> List[Tuple[dict, dict]]:
        out = []
        for lm in self.landmarks():
            for t in lm.get("tasks") or []:
                if isinstance(t, dict) and t.get("id") and t.get("name") and t.get("model") and t.get("task"):
                    out.append((lm, t))
        return out

    def find(self, landmark_id: str, task_id: str) -> Optional[Tuple[dict, dict]]:
        for lm, t in self.tasks():
            if lm["id"] == landmark_id and t["id"] == task_id:
                return lm, t
        return None


def match_task(text: str, tasks: List[Tuple[dict, dict]], cutoff: float = 0.78) -> Optional[Tuple[dict, dict]]:
    """The task whose spoken name the utterance is (or contains): "could you pick up tigger please" →
    "pick up tigger". Containment first (longest name wins), then a close fuzzy match of the whole
    stripped utterance (Whisper's "pick up tiger")."""
    q = normalise(text)
    if not q or not tasks or _QUESTION.match((text or "").lower().strip()):
        return None
    names = [(normalise(t["name"]), lm, t) for lm, t in tasks]
    names = [n for n in names if n[0]]
    hits = [n for n in names if n[0] == q or re.search(r"(?<![a-z])" + re.escape(n[0]) + r"(?![a-z])", q)]
    if hits:
        return max(hits, key=lambda n: len(n[0]))[1:]
    close = difflib.get_close_matches(q, [n[0] for n in names], n=1, cutoff=cutoff)
    if close:
        return next(n[1:] for n in names if n[0] == close[0])
    return None


def vla_action_command(vla_dir: str, task: dict, landmark_name: str = "") -> List[str]:
    """Same shape as rd_ws/Scripts/path.py's `vla:` post_action: run.sh when the checkout has it, else the venv."""
    vla_dir = os.path.expanduser(vla_dir)
    run_sh = os.path.join(vla_dir, "run.sh")
    if os.path.exists(run_sh):
        cmd = [run_sh, "scripts/vla_action.py"]
    else:
        py = os.path.join(vla_dir, ".venv", "bin", "python")
        cmd = [py if os.path.exists(py) else "python3", os.path.join(vla_dir, "scripts", "vla_action.py")]
    cmd += ["--model", str(task["model"]), "--task", str(task["task"]), "--seconds", str(float(task.get("seconds") or 120))]
    if task.get("still"):
        cmd += ["--still", str(float(task["still"]))]
    if task.get("min_seconds") is not None and task.get("min_seconds") != "":
        cmd += ["--min-seconds", str(float(task["min_seconds"]))]
    if task.get("keep_grasp"):
        cmd.append("--keep-grasp")
    if task.get("home", True) is False:
        cmd.append("--no-home")
    if task.get("still_mode") in ("fingers",):
        cmd += ["--still-mode", str(task["still_mode"])]
    if task.get("done_when") in ("closed", "open"):
        cmd += ["--done-when", str(task["done_when"])]
    if landmark_name:
        cmd += ["--landmark", landmark_name]
    if task.get("extra"):
        cmd += shlex.split(str(task["extra"]))
    return cmd


class TaskRunner:
    """Drives to a task's landmark, runs its policy, talks about it. `navigate(landmark) -> (ok, why)` blocks."""

    def __init__(self, store: LandmarkStore, navigate: Callable[[dict], Tuple[bool, str]], say: Callable[[str], None],
                 log: Optional[Callable[[str], None]] = None, vla_dir: str = "~/rd_ws/VLA", run_timeout: float = 600.0,
                 on_done: Optional[Callable[[dict, dict, int, str], None]] = None):
        self.store = store
        self.navigate = navigate
        self.say = say
        self._log = log or (lambda m: None)
        self.vla_dir = vla_dir
        self.run_timeout = run_timeout
        self.on_done = on_done
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._proc: Optional[subprocess.Popen] = None
        self._current: Optional[Tuple[dict, dict]] = None
        self._phase = ""              # driving | running | ""
        self._cancel = threading.Event()
        self.last: dict = {}          # last run: landmark, task, rc, reason, when

    # ------------------------------------------------------------------ what exists
    def names(self) -> List[str]:
        return [f"{t['name']} (at the {lm['name']})" for lm, t in self.store.tasks()]

    def match(self, text: str) -> Optional[Tuple[dict, dict]]:
        return match_task(text, self.store.tasks())

    @property
    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def status(self) -> str:
        cur = self._current
        if not self.busy or cur is None:
            if self.last:
                return f"The last job was {self.last['task']} at the {self.last['landmark']}: {self.last['reason']}."
            return "I'm not working on anything."
        lm, t = cur
        return (f"I'm driving to the {lm['name']} to {t['name']}." if self._phase == "driving"
                else f"I'm at the {lm['name']} doing {t['name']}.")

    # ------------------------------------------------------------------ starting
    def run(self, text: str) -> str:
        """Start the task the utterance names; the reply is what the robot says right away."""
        hit = self.match(text)
        if hit is None:
            known = ", ".join(t["name"] for _lm, t in self.store.tasks())
            return f"I don't know how to {normalise(text) or 'do that'}. " + (f"I can {known}." if known else "No jobs are set up yet.")
        return self.start(*hit)

    def run_ids(self, landmark_id: str, task_id: str) -> str:
        hit = self.store.find(landmark_id, task_id)
        if hit is None:
            return f"There is no task {task_id} at {landmark_id}."
        return self.start(*hit)

    def start(self, lm: dict, task: dict) -> str:
        with self._lock:
            if self.busy and self._current is not None:
                clm, ct = self._current
                return f"I'm busy: {ct['name']} at the {clm['name']}. Say stop first."
            self._cancel.clear()
            self._current = (lm, task)
            self._phase = "driving"
            self._thread = threading.Thread(target=self._worker, args=(lm, task), daemon=True, name="landmark-task")
            self._thread.start()
        return f"Heading to the {lm['name']} to {task['name']}."

    # ------------------------------------------------------------------ the job
    def _worker(self, lm: dict, task: dict) -> None:
        rc, reason = -1, "unknown"
        try:
            self._log(f"task {task['name']!r}: driving to {lm['name']!r} ({lm.get('x')}, {lm.get('y')}, {lm.get('yaw')})")
            ok, why = self.navigate(lm)
            if self._cancel.is_set():
                rc, reason = 5, "cancelled on the way"
                return
            if not ok:
                rc, reason = 2, f"could not get there ({why})"
                self.say(f"I couldn't get to the {lm['name']}: {why}.")
                return
            self._phase = "running"
            self.say(f"I'm at the {lm['name']}. Here goes: {task['name']}.")
            rc, reason = self._run_policy(lm, task)
            if self._cancel.is_set():
                reason = "stopped"
                self.say(f"Stopped {task['name']}.")
            elif rc == 0:
                self.say(f"Done with {task['name']}.")
            else:
                self.say(f"I couldn't finish {task['name']}: {reason}.")
        except Exception as e:  # noqa: BLE001
            rc, reason = -1, f"{type(e).__name__}: {e}"
            self._log(f"task {task['name']!r} crashed: {reason}")
            self.say(f"Something went wrong with {task['name']}.")
        finally:
            self.last = {"landmark": lm["name"], "task": task["name"], "rc": rc, "reason": reason, "when": time.time()}
            self._log(f"task {task['name']!r} at {lm['name']!r}: exit {rc} — {reason}")
            self._phase, self._current, self._proc = "", None, None
            if self.on_done:
                try:
                    self.on_done(lm, task, rc, reason)
                except Exception:  # noqa: BLE001
                    pass

    def _run_policy(self, lm: dict, task: dict) -> Tuple[int, str]:
        cmd = vla_action_command(self.vla_dir, task, lm.get("name", ""))
        cwd = os.path.expanduser(self.vla_dir)
        self._log("task run: " + shlex.join(cmd))
        try:
            proc = subprocess.Popen(cmd, cwd=cwd if os.path.isdir(cwd) else None, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
        except OSError as e:
            return 4, f"could not start the run ({e})"
        self._proc = proc
        last_action, last_error = "", ""
        deadline = time.monotonic() + float(task.get("seconds") or 120) + self.run_timeout

        def _reaper():
            while proc.poll() is None:
                if time.monotonic() > deadline:
                    self._log("task run: over time; terminating")
                    self._terminate(proc)
                    return
                time.sleep(1.0)
        threading.Thread(target=_reaper, daemon=True).start()
        for line in proc.stdout:
            line = line.rstrip()
            if not line:
                continue
            if line.startswith("[action]") or line.startswith("[auto]") or "error" in line.lower():
                self._log("vla_action: " + line)
            if line.startswith("[action]"):
                last_action = line[len("[action]"):].strip()
            elif re.match(r"^\w*(Error|Exception)\b", line):      # a traceback's last line: the real reason
                last_error = line
        rc = proc.wait()
        if rc == 0:
            m = re.search(r"done:\s*(.+?)\s+—", last_action)
            return 0, (m.group(1) if m else "done")
        reason = {3: "the arm faulted", 4: "the setup failed", 5: "it never started"}.get(rc, f"exit code {rc}")
        m = re.search(r"FAILED:\s*(.+?)\s+—", last_action)
        if m and not m.group(1).startswith("exit "):
            reason = m.group(1)
        elif last_error:
            reason = re.sub(r"^\w+:\s*", "", last_error).split(" — ")[0][:140]
        elif rc < 0:
            reason = "stopped"
        return rc, reason

    @staticmethod
    def _terminate(proc: subprocess.Popen) -> None:
        """SIGTERM to the whole group: eval_hw maps it to EXIT (hand open, arm home, UR down) while
        vla_action.py itself just dies — the run is over either way."""
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            pass

    # ------------------------------------------------------------------ stopping
    def stop(self) -> str:
        if not self.busy or self._current is None:
            return ""
        lm, t = self._current
        self._cancel.set()
        phase = self._phase
        proc = self._proc
        if proc is not None and proc.poll() is None:
            self._terminate(proc)
        return (f"Stopping on the way to the {lm['name']}." if phase == "driving"
                else f"Stopping {t['name']}; the arm is going home.")


class NoTaskRunner:
    busy = False

    def names(self) -> List[str]:
        return []

    def match(self, text: str):
        return None

    def run(self, text: str) -> str:
        return "No arm jobs are set up."

    def start(self, lm: dict, task: dict) -> str:
        return "No arm jobs are set up."

    def stop(self) -> str:
        return ""

    def status(self) -> str:
        return "I'm not working on anything."


__all__ = ["LandmarkStore", "TaskRunner", "NoTaskRunner", "match_task", "normalise", "vla_action_command"]
