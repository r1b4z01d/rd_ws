"""Play a saved pendant program on the UR arm through its Dashboard Server (TCP 29999).

Used by the face node to run a gesture program (``wave.urp``, recorded on the teach pendant)
when it recognises someone. The dashboard only loads and starts programs the pendant already
knows, so nothing here can move the arm along a path the user did not program.

Preconditions checked before every play (any failure is reported, never forced):
  * PolyScope is in Remote Control (ur_rtde needs that too, so it normally is),
  * robot mode RUNNING (powered, brakes released) and safety status NORMAL,
  * no program or external-control script running (the VLA teleop / eval own the arm then).
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional

DEFAULT_UR_IP = "192.168.11.21"


class UrDashboard:
    def __init__(self, ip: str = DEFAULT_UR_IP, port: int = 29999, timeout: float = 3.0,
                 log: Optional[Callable[[str], None]] = None):
        self.ip, self.port, self.timeout = ip, int(port), float(timeout)
        self._log = log or (lambda m: None)
        self._lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._buf = b""
        self.busy = False
        self.last_result = ""

    # -- transport ---------------------------------------------------------------
    def _connect(self) -> None:
        self._sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        self._buf = b""
        self._readline()                       # "Connected: Universal Robots Dashboard Server"

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.sendall(b"quit\n")
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def _readline(self) -> str:
        assert self._sock is not None
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("dashboard closed the connection")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8", "replace").strip()

    def _cmd(self, cmd: str) -> str:
        assert self._sock is not None
        self._sock.sendall((cmd + "\n").encode())
        return self._readline()

    # -- queries -----------------------------------------------------------------
    def status(self) -> dict:
        """Snapshot of the dashboard state (opens its own connection)."""
        with self._lock:
            try:
                self._connect()
                st = {
                    "remote": self._cmd("is in remote control").lower() == "true",
                    "robotmode": self._cmd("robotmode").replace("Robotmode:", "").strip(),
                    "safety": self._cmd("safetystatus").replace("Safetystatus:", "").strip(),
                    "running": self._cmd("running").lower().endswith("true"),
                    "loaded": self._cmd("get loaded program").replace("Loaded program:", "").strip(),
                    "state": self._cmd("programState"),
                }
            finally:
                self._close()
        return st

    # -- power up (the eval/teleop startup) ------------------------------------------
    def _one(self, cmd: str) -> str:
        """One command on its own connection (used by the slow power-up waits)."""
        with self._lock:
            try:
                self._connect()
                return self._cmd(cmd)
            finally:
                self._close()

    @staticmethod
    def _value(reply: str) -> str:
        return reply.split(":", 1)[-1].strip()

    def _wait(self, cmd: str, want: set, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._value(self._one(cmd)) in want:
                return True
            time.sleep(1.0)
        return False

    def power_up(self, timeout: float = 60.0) -> str:
        """Bring the arm to RUNNING (brakes released): the same startup eval_hw and the teleop panel
        run through VLA's ur_power.power_up, kept in step with it. Remote Control is required.
        FAULT/VIOLATION -> close safety popup + restart safety; PROTECTIVE_STOP -> close safety popup
        + unlock protective stop; then power on, brake release. Returns a status line; raises
        RuntimeError when the arm does not get there. Already RUNNING is a no-op. It releases the
        brakes without knowing whether the arm is clear, exactly like the eval does."""
        try:
            if self._one("is in remote control").lower() != "true":
                raise RuntimeError("pendant is not in Remote Control mode; switch it on the teach pendant")
            if self._value(self._one("robotmode")) == "RUNNING" and self._value(self._one("safetystatus")) == "NORMAL":
                return "UR already RUNNING"
            note = ""
            ss = self._value(self._one("safetystatus"))
            if ss in ("FAULT", "VIOLATION"):
                note = f"restarted the safety system after {ss}; "
                self._log(f"UR safety status {ss}: a safety limit was violated or a joint could not stop in time "
                          "(see the pendant log); restarting safety via the dashboard like eval_hw does")
                self._one("close safety popup")
                reply = self._one("restart safety")
                if not self._wait("safetystatus", {"NORMAL"}, 45):
                    raise RuntimeError(f"safety did not return to NORMAL after 'restart safety' ({reply}); now "
                                       f"{self._one('safetystatus')}; restart it on the pendant")
                ss = "NORMAL"
            if ss == "PROTECTIVE_STOP":
                note = "cleared protective stop; "
                self._log("UR in PROTECTIVE_STOP: clearing it via the dashboard like eval_hw does "
                          "(repeats after clean exits => encoder/safety fault, not something to paper over)")
                self._one("close safety popup")
                self._one("unlock protective stop")
                if not self._wait("safetystatus", {"NORMAL"}, 15):
                    raise RuntimeError("safety status did not return to NORMAL: " + self._one("safetystatus"))
            if self._value(self._one("robotmode")) != "RUNNING":
                self._one("power on")
                if not self._wait("robotmode", {"IDLE", "RUNNING"}, timeout):
                    raise RuntimeError("timed out waiting for IDLE: " + self._one("robotmode"))
                self._one("brake release")
                if not self._wait("robotmode", {"RUNNING"}, timeout):
                    raise RuntimeError("timed out waiting for RUNNING: " + self._one("robotmode"))
                note += "powered up (brakes released)"
            return note.rstrip("; ") or "UR already RUNNING"
        except OSError as e:
            raise RuntimeError(f"UR dashboard unreachable at {self.ip}:{self.port} ({e}); is the controller on?") from e

    # -- play --------------------------------------------------------------------
    def play(self, program: str, wait: bool = True, timeout: float = 40.0,
             auto_power_up: bool = False) -> tuple[bool, str]:
        """Load ``program`` (a name under /programs, .urp optional) and start it. With ``wait``
        this blocks until the program has finished (or stops it at ``timeout``). Returns
        (ok, message); the message is meant for the log."""
        name = program.strip()
        if not name:
            return False, "no program configured"
        if not name.endswith(".urp"):
            name += ".urp"
        if not self._lock.acquire(blocking=False):
            return False, "dashboard busy"
        self.busy = True
        t0 = time.monotonic()
        note = ""
        try:
            if auto_power_up:
                try:
                    st = self.status()
                except OSError as e:
                    return False, f"cannot reach the UR dashboard at {self.ip}:{self.port} ({e})"
                if st["remote"] and (st["robotmode"] != "RUNNING" or st["safety"] != "NORMAL"):
                    self._lock.release()            # power_up uses the lock per command
                    try:
                        note = self.power_up() + "; "
                    except RuntimeError as e:
                        return False, f"arm startup failed: {e}"
                    finally:
                        self._lock.acquire()
            try:
                self._connect()
            except OSError as e:
                return False, f"cannot reach the UR dashboard at {self.ip}:{self.port} ({e})"
            if self._cmd("is in remote control").lower() != "true":
                return False, "PolyScope is in Local Control; switch the pendant to Remote Control"
            mode = self._cmd("robotmode").replace("Robotmode:", "").strip()
            if mode != "RUNNING":
                return False, f"robot mode is {mode} (needs RUNNING: powered on, brakes released)"
            safety = self._cmd("safetystatus").replace("Safetystatus:", "").strip()
            if safety != "NORMAL":
                return False, f"safety status is {safety}"
            if self._cmd("running").lower().endswith("true"):
                return False, f"a program is already running ({self._cmd('programState')}); arm is busy"
            r = self._cmd(f"load {name}")
            if not r.lower().startswith("loading program"):
                return False, f"load {name}: {r}"
            r = self._cmd("play")
            if not r.lower().startswith("starting program"):
                return False, f"play: {r}"
            if not wait:
                return True, f"{name} started"
            # Starting takes a moment: wait for PLAYING (up to 4 s), then for it to end.
            seen_playing = False
            while True:
                time.sleep(0.25)
                st = self._cmd("programState")
                el = time.monotonic() - t0
                if st.startswith("PLAYING"):
                    seen_playing = True
                elif seen_playing or el > 4.0:
                    if st.startswith("PAUSED"):
                        self._cmd("stop")
                        return False, f"{name} paused itself (popup or halt on the pendant?) after {el:.1f}s; stopped"
                    return True, f"{note}{name} played in {el:.1f}s"
                if el > timeout:
                    self._cmd("stop")
                    return False, f"{name} still running after {timeout:.0f}s; stopped it"
        except (OSError, ConnectionError) as e:
            return False, f"dashboard error: {e}"
        finally:
            self._close()
            self.busy = False
            self._lock.release()

    def stop(self) -> str:
        with self._lock:
            try:
                self._connect()
                return self._cmd("stop")
            except OSError as e:
                return f"dashboard error: {e}"
            finally:
                self._close()


if __name__ == "__main__":          # python3 -m robot_face.ur_dashboard [ip] [program]
    import sys
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_UR_IP
    d = UrDashboard(ip, log=print)
    print(d.status())
    if len(sys.argv) > 2:
        print(d.play(sys.argv[2]))
