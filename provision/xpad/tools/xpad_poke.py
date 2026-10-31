#!/usr/bin/env python3
"""Send the Xbox-360 LED command to an XInput receiver's interface 0 via usbfs and dump what the
interrupt-IN endpoint returns for a while; the kernel driver is detached first and re-attached
after. Found the 8BitDo 2.4 GHz receiver's "silent until LED command" behaviour (2026-10-31).

    sudo python3 xpad_poke.py /dev/bus/usb/001/032 10
"""
import ctypes, errno, fcntl, os, sys, time

dev = sys.argv[1]            # /dev/bus/usb/<bus>/<devnum>  (lsusb -d 2dc8:)
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10
IFNO, EP_OUT, EP_IN = 0, 0x05, 0x84   # the receiver's XInput interface and endpoints (lsusb -v)


def IOC(d, t, n, s):
    return (d << 30) | (s << 16) | (ord(t) << 8) | n


NONE, W, R = 0, 1, 2


class UsbdevfsIoctl(ctypes.Structure):
    _fields_ = [("ifno", ctypes.c_int), ("ioctl_code", ctypes.c_int), ("data", ctypes.c_void_p)]


class Bulk(ctypes.Structure):
    _fields_ = [("ep", ctypes.c_uint), ("len", ctypes.c_uint), ("timeout", ctypes.c_uint),
                ("data", ctypes.c_void_p)]


CLAIM = IOC(R, 'U', 15, 4)
RELEASE = IOC(R, 'U', 16, 4)
IOCTL = IOC(R | W, 'U', 18, ctypes.sizeof(UsbdevfsIoctl))
DISCONNECT = IOC(NONE, 'U', 22, 0)
CONNECT = IOC(NONE, 'U', 23, 0)
BULK = IOC(R | W, 'U', 2, ctypes.sizeof(Bulk))   # usbfs runs this as an interrupt transfer on int endpoints

fd = os.open(dev, os.O_RDWR)


def drv(code):
    s = UsbdevfsIoctl(IFNO, code, None)
    try:
        fcntl.ioctl(fd, IOCTL, s)
        return "ok"
    except OSError as e:
        return os.strerror(e.errno)


def xfer(ep, data=None, n=32, timeout=1000):
    raw = bytes(data) if data else bytes(n)
    buf = ctypes.create_string_buffer(raw, len(raw))
    b = Bulk(ep, len(raw), timeout, ctypes.addressof(buf))
    try:
        r = fcntl.ioctl(fd, BULK, b)
    except OSError as e:
        if e.errno == errno.ETIMEDOUT:
            return None
        return e
    return buf.raw[:r]


print("detach kernel driver from if0:", drv(DISCONNECT))
try:
    fcntl.ioctl(fd, CLAIM, ctypes.c_uint(IFNO))
    print("claim if0: ok")
except OSError as e:
    print("claim if0:", os.strerror(e.errno))
    sys.exit(1)
try:
    print("OUT led 01 03 02 ->", xfer(EP_OUT, [0x01, 0x03, 0x02]))
    print("OUT rumble off   ->", xfer(EP_OUT, [0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    t0 = time.time()
    n = 0
    last = None
    while time.time() - t0 < secs:
        p = xfer(EP_IN, timeout=500)
        if p is None:
            continue
        if isinstance(p, OSError):
            print("IN error:", p)
            time.sleep(0.2)
            continue
        n += 1
        if p != last:
            print(f"{time.time() - t0:5.1f}s IN[{len(p)}]: {p.hex(' ')}")
            last = p
    print(f"IN packets in {secs:.0f}s: {n}")
finally:
    try:
        fcntl.ioctl(fd, RELEASE, ctypes.c_uint(IFNO))
    except OSError as e:
        print("release:", os.strerror(e.errno))
    print("re-attach kernel driver:", drv(CONNECT))
