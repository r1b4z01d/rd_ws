"""Wiggle the AmazingHand's fingers while the arm waves (thumb held still).

The hand is an ESP32 on the robot WiFi speaking the VLA (rd_ws/VLA) line protocol on UDP :8766:
``J:<8 servo offsets deg>,<speed>\\n`` (one datagram per line; VLA ur5e_lerobot/hand/amazing_hand_client.py
is the reference — the finger→servo mapping tables below are copied from it verbatim). UDP so a lost
setpoint is simply superseded by the next one; a ``Q`` probe (``K:<uptime>`` answer) before each
wiggle tells a dead hand from a silent network.

The pattern: index, middle and ring curl between open and ``curl`` with a sine at ``hz``, each finger
``phase`` of a period behind the previous one (a wave running from the index to the ring), the thumb
stays at ``thumb``. On stop the fingers open.
"""
from __future__ import annotations

import math
import socket
import threading
import time
from typing import Callable, Optional

FINGER_ORDER = ("index", "middle", "ring", "thumb")
FINGER_TO_SERVO = {"index": (0, 1), "middle": (2, 3), "ring": (4, 5), "thumb": (6, 7)}
JOINT_OPEN_OFFSETS = (-35.0, 35.0, -35.0, 35.0, -35.0, 35.0, -35.0, 35.0)
JOINT_CLOSE_OFFSETS = (90.0, -90.0, 90.0, -90.0, 90.0, -90.0, 70.0, -70.0)
DEFAULT_HAND_HOST = "192.168.11.117"
DEFAULT_HAND_PORT = 8766


def curls_to_offsets(curls) -> list:
    """[index, middle, ring, thumb] curls in 0..1 -> 8 servo offsets (deg)."""
    out = [0.0] * 8
    for finger, c in zip(FINGER_ORDER, curls):
        c = 0.0 if c < 0.0 else 1.0 if c > 1.0 else float(c)
        for idx in FINGER_TO_SERVO[finger]:
            o, cl = JOINT_OPEN_OFFSETS[idx], JOINT_CLOSE_OFFSETS[idx]
            out[idx] = o + (cl - o) * c
    return out


def joint_line(offsets, speed: int) -> bytes:
    return ("J:" + ",".join(f"{float(v):.2f}" for v in offsets) + f",{int(speed)}\n").encode("ascii")


class HandWiggle:
    def __init__(self, host: str = DEFAULT_HAND_HOST, port: int = DEFAULT_HAND_PORT, speed: int = 500,
                 hz: float = 0.5, curl: float = 0.333, thumb: float = 0.0, rate: float = 15.0,
                 phase: float = 0.25, log: Optional[Callable[[str], None]] = None):
        self.host, self.port = str(host), int(port)
        self.speed = int(speed)            # firmware J: speed, 50..2000
        self.hz = float(hz)                # ripples per second
        self.curl = float(curl)            # how far the fingers close (0 open .. 1 closed)
        self.thumb = float(thumb)          # the thumb's fixed curl
        self.rate = float(rate)            # setpoints per second
        self.phase = float(phase)          # fraction of a period between neighbouring fingers (0.25 = a quarter cycle)
        self._log = log or (lambda m: None)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._last_warn = 0.0
        self.sent = 0

    # -- transport -----------------------------------------------------------------
    def _socket(self) -> socket.socket:
        # Unconnected on purpose: the hand's firmware answers from a different source port than it
        # listens on, and a connected UDP socket silently drops such replies (the probe never saw the K:).
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.6)
        return s

    def probe(self) -> bool:
        """``Q`` -> ``K:<uptime>``: the hand is powered, on WiFi and listening."""
        try:
            s = self._socket()
            s.sendto(b"Q\n", (self.host, self.port))
            data, _ = s.recvfrom(256)
            s.close()
            return data.startswith(b"K")
        except OSError:
            return False

    def send_curls(self, curls, sock: Optional[socket.socket] = None) -> None:
        own = sock is None
        try:
            s = sock or self._socket()
            s.sendto(joint_line(curls_to_offsets(curls), self.speed), (self.host, self.port))
            self.sent += 1
            if own:
                s.close()
        except OSError as e:
            now = time.monotonic()
            if now - self._last_warn > 30.0:
                self._last_warn = now
                self._log(f"hand {self.host}:{self.port}: send failed ({e})")

    # -- the wiggle ------------------------------------------------------------------
    def start(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            return True
        if not self.probe():
            now = time.monotonic()
            if now - self._last_warn > 30.0:
                self._last_warn = now
                self._log(f"hand {self.host}:{self.port} does not answer the Q probe; no finger wiggle")
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="hand_wiggle")
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive() and threading.current_thread() is not t:
            t.join(timeout=1.0)
        self._thread = None

    def _run(self) -> None:
        s = self._socket()
        t0 = time.monotonic()
        dt = 1.0 / max(1.0, self.rate)
        try:
            while not self._stop.is_set():
                t = time.monotonic() - t0
                lag = self.phase / max(0.05, self.hz)      # seconds between neighbouring fingers

                def c(delay):
                    x = max(0.0, t - delay)                 # each finger starts open and joins in turn
                    return self.curl * (0.5 - 0.5 * math.cos(2.0 * math.pi * self.hz * x))
                self.send_curls([c(0.0), c(lag), c(2 * lag), self.thumb], s)
                self._stop.wait(dt)
        finally:
            for _ in range(2):           # fingers open again (twice: UDP)
                self.send_curls([0.0, 0.0, 0.0, self.thumb], s)
                time.sleep(0.05)
            s.close()


if __name__ == "__main__":      # python3 -m robot_face.hand_wiggle [seconds] [host]
    import sys
    w = HandWiggle(sys.argv[2] if len(sys.argv) > 2 else DEFAULT_HAND_HOST, log=print)
    print("probe:", w.probe())
    if w.start():
        time.sleep(float(sys.argv[1]) if len(sys.argv) > 1 else 4.0)
        w.stop()
        print("sent", w.sent, "setpoints")
