import os, sys, termios, select, time
dev = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyTHS1'
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd); a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
a[4] = a[5] = termios.B115200; termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
msg = b'RD-UART-LOOPBACK-0123456789'; got = b''
os.write(fd, msg); t0 = time.time()
while time.time() - t0 < 1.0 and len(got) < len(msg):
    r, _, _ = select.select([fd], [], [], 0.1)
    if r:
        try: got += os.read(fd, 256)
        except BlockingIOError: pass
print('LOOPBACK OK: pins 8 and 10 are ttyTHS1' if got == msg else f'no echo (read {len(got)} bytes: {got[:40]!r})')
