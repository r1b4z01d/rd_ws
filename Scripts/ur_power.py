#!/usr/bin/env python3
"""Power the UR5e up or down over the Dashboard Server (port 29999), no VNC.

  ur_power.py up      # clear protective stop, power on, release brakes -> RUNNING
  ur_power.py down    # brake-safe power off, then shut down the controller
  ur_power.py status

Why this is needed: if the safety system sees a large joint change since the
last power-off it raises C161A0 ("Large movement of the robot detected while it
was powered off") and drops into PROTECTIVE_STOP, which PolyScope shows as a
popup that must be accepted. The dashboard can clear it with
`unlock protective stop` (verified on PolyScope 5.21.2, 2026-10-16).

C161A0 is usually caused by cutting controller power instead of shutting down,
so the last joint positions never get saved. Use `down` rather than pulling
power. Every auto-clear is logged: if it fires after clean shutdowns it is a
real encoder/safety-board fault, not something to keep clearing.

Needs the pendant in Remote Control mode. Clear the arm's workspace first:
`up` releases the brakes and nothing here checks that the arm is clear.
"""
import argparse
import socket
import sys
import time

UR_IP = "192.168.11.21"
DASHBOARD_PORT = 29999
TIMEOUT = 60.0


def dashboard(cmd, ip=UR_IP):
    with socket.create_connection((ip, DASHBOARD_PORT), timeout=5) as s:
        s.recv(1024)  # "Connected: Universal Robots Dashboard Server"
        s.sendall((cmd + "\n").encode())
        return s.recv(1024).decode().strip()


def value(reply):
    return reply.split(":", 1)[-1].strip()


def wait_for(cmd, want, timeout=TIMEOUT, ip=UR_IP):
    end = time.time() + timeout
    while time.time() < end:
        if value(dashboard(cmd, ip)) in want:
            return True
        time.sleep(1)
    return False


def status(ip):
    print("remote control:", dashboard("is in remote control", ip))
    print("robotmode     :", value(dashboard("robotmode", ip)))
    print("safetystatus  :", value(dashboard("safetystatus", ip)))


def up(ip):
    if dashboard("is in remote control", ip) != "true":
        sys.exit("Pendant is not in Remote Control mode; switch it on the teach pendant.")
    if value(dashboard("robotmode", ip)) == "RUNNING":
        print("already RUNNING")
        return

    if value(dashboard("safetystatus", ip)) == "PROTECTIVE_STOP":
        print("WARNING: PROTECTIVE_STOP at startup (likely C161A0, joint-position "
              "mismatch after power-off); clearing. If this happens after clean "
              "shutdowns, suspect an encoder/safety fault.")
        dashboard("close safety popup", ip)
        print(dashboard("unlock protective stop", ip))
        if not wait_for("safetystatus", {"NORMAL"}, 15, ip):
            sys.exit("Safety status did not return to NORMAL: " + dashboard("safetystatus", ip))

    print(dashboard("power on", ip))
    if not wait_for("robotmode", {"IDLE", "RUNNING"}, ip=ip):
        sys.exit("Timed out waiting for IDLE: " + dashboard("robotmode", ip))
    print(dashboard("brake release", ip))
    if not wait_for("robotmode", {"RUNNING"}, ip=ip):
        sys.exit("Timed out waiting for RUNNING: " + dashboard("robotmode", ip))
    print("RUNNING")


def down(ip):
    print(dashboard("power off", ip))
    wait_for("robotmode", {"POWER_OFF"}, 30, ip)
    # Shutting down the controller lets the safety system save joint positions.
    print(dashboard("shutdown", ip))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("action", choices=["up", "down", "status"])
    p.add_argument("--ip", default=UR_IP)
    args = p.parse_args()
    {"up": up, "down": down, "status": status}[args.action](args.ip)


if __name__ == "__main__":
    main()
