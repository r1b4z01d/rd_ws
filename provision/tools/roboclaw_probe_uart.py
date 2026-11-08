import os, sys, termios, select, time, struct
dev = sys.argv[1]; baud = int(sys.argv[2]); addr = int(sys.argv[3]) if len(sys.argv) > 3 else 128
speed = getattr(termios, 'B%d' % baud)
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
a[4] = speed; a[5] = speed
termios.tcsetattr(fd, termios.TCSANOW, a)
def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
def ask(cmd, nbytes, timeout=0.4):
    termios.tcflush(fd, termios.TCIOFLUSH)
    os.write(fd, bytes([addr, cmd]))
    buf = b''; t0 = time.time()
    while time.time() - t0 < timeout and (nbytes is None or len(buf) < nbytes):
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try: buf += os.read(fd, 256)
            except BlockingIOError: pass
            if nbytes is None and b'\x00' in buf and len(buf) >= buf.index(b'\x00') + 3: break
    return buf
v = ask(21, None)
if not v:
    print(f'{baud}: no reply'); sys.exit(1)
s = v[:v.index(b'\x00')] if b'\x00' in v else v
ok = len(v) >= len(s)+3 and crc16(bytes([addr, 21]) + v[:len(s)+1]) == struct.unpack('>H', v[len(s)+1:len(s)+3])[0]
print(f'{baud}: version {s.decode(errors="replace").strip()!r}', 'crc ok' if ok else f'crc BAD raw={v!r}')
if not ok: sys.exit(2)
for name, cmd in (('main battery', 24), ('logic battery', 25)):
    r = ask(cmd, 4)
    print(f'  {name}: {struct.unpack(">H", r[:2])[0] / 10:.1f} V' if len(r) >= 2 else f'  {name}: no reply')
r = ask(90, 6)
if len(r) >= 4:
    err = struct.unpack('>I', r[:4])[0]; print('  status: 0x%08x' % err, 'normal' if err == 0 else '')
r = ask(49, 6)
if len(r) >= 4: print('  motor currents: %.2f A / %.2f A' % tuple(x/100 for x in struct.unpack('>hh', r[:4])))
good = bad = 0
for _ in range(20):
    r = ask(24, 4); 
    if len(r) >= 4: good += 1
    else: bad += 1
print(f'  20 reads: {good} ok, {bad} failed')
