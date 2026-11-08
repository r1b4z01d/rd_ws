import os, sys, termios, select, time
baud = int(sys.argv[1]); fd = os.open("/dev/ttyTHS1", os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
a = termios.tcgetattr(fd); a[0]=termios.INPCK|termios.PARMRK; a[1]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL; a[3]=0; a[4]=a[5]=getattr(termios, 'B%d' % baud)
termios.tcsetattr(fd, termios.TCSANOW, a)
ZERO = b"\x00"; MARK = b"\xff\x00"; EXPECT = b"USB Roboclaw 2x15a v4.3.6\n\x00"
def cap(secs):
    buf=b""; t0=time.time()
    while time.time()-t0<secs:
        r,_,_=select.select([fd],[],[],0.05)
        if r:
            try: buf+=os.read(fd,512)
            except BlockingIOError: pass
    return buf
good = 0
for i in range(5):
    termios.tcflush(fd, termios.TCIOFLUSH); os.write(fd, bytes([128, 21])); r = cap(0.4)
    ok = r.startswith(EXPECT); good += ok
    zeros = max(r.count(ZERO) - 1, 0); marks = r.count(MARK); verdict = "OK" if ok else "CORRUPT"
    print(f"  {baud} req {i}: {len(r)} bytes, zeros={zeros}, errmarks={marks}, {verdict} {r[:32]!r}")
print(f"  {baud}: {good}/5 clean")
