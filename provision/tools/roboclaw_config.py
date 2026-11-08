"""RoboClaw packet-serial config tool (USB CDC or UART). Read-only unless --set/--timeout/--save given.
usage: roboclaw_config.py DEV [--baud N] [--addr 128]
                          [--set-mode packet --set-baud 115200 --set-addr 128] [--timeout TENTHS] [--save]
"""
import os, sys, termios, select, time, struct, argparse
p = argparse.ArgumentParser()
p.add_argument('dev'); p.add_argument('--baud', type=int, default=0, help='UART line speed (0 = leave, for USB CDC)')
p.add_argument('--addr', type=int, default=128)
p.add_argument('--set-mode', choices=['rc', 'analog', 'simple', 'packet'])
p.add_argument('--set-baud', type=int, choices=[2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800])
p.add_argument('--set-addr', type=int, choices=range(128, 136))
p.add_argument('--timeout', type=int, help='serial timeout in 100 ms units (0-255, 0 = off)')
p.add_argument('--save', action='store_true', help='write settings to EEPROM (cmd 94)')
args = p.parse_args(); addr = args.addr
fd = os.open(args.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
if args.baud: a[4] = a[5] = getattr(termios, 'B%d' % args.baud)
termios.tcsetattr(fd, termios.TCSANOW, a)
def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8): crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
def xfer(payload, nbytes, timeout=0.6):
    termios.tcflush(fd, termios.TCIOFLUSH); os.write(fd, payload)
    buf = b''; t0 = time.time()
    while time.time() - t0 < timeout and (nbytes is None or len(buf) < nbytes):
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try: buf += os.read(fd, 256)
            except BlockingIOError: pass
            if nbytes is None and b'\x00' in buf and len(buf) >= buf.index(b'\x00') + 3: break
    return buf
def read(cmd, nbytes):               # reads: [addr, cmd] -> data + crc16 over (addr, cmd, data)
    r = xfer(bytes([addr, cmd]), nbytes)
    if len(r) < nbytes: raise SystemExit(f'cmd {cmd}: no/short reply {r!r}')
    if crc16(bytes([addr, cmd]) + r[:-2]) != struct.unpack('>H', r[-2:])[0]: raise SystemExit(f'cmd {cmd}: bad CRC {r!r}')
    return r[:-2]
def write(cmd, data=b''):             # writes: [addr, cmd, data, crc16] -> 0xFF
    pkt = bytes([addr, cmd]) + data; pkt += struct.pack('>H', crc16(pkt))
    r = xfer(pkt, 1)
    if r[:1] != b'\xff': raise SystemExit(f'cmd {cmd}: not acknowledged ({r!r})')
MODES = ['RC', 'Analog', 'Simple Serial', 'Packet Serial']
BAUDS = [2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800]
def show(cfg, label):
    mode = cfg & 3
    print(f'{label}: config 0x{cfg:04x} -> mode {MODES[mode]}', end='')
    if mode >= 2: print(f', baud {BAUDS[(cfg >> 5) & 7]}, address 0x{0x80 + ((cfg >> 8) & 7):02x}', end='')
    else: print(f', mixing={bool(cfg & 0x20)} exp={bool(cfg & 0x40)} mcu={bool(cfg & 0x80)} flip={bool(cfg & 0x100)}', end='')
    print(f', battery mode {(cfg >> 2) & 7}, slave={bool(cfg & 0x800)} relay={bool(cfg & 0x1000)} swapenc={bool(cfg & 0x2000)} swapbtn={bool(cfg & 0x4000)} multi={bool(cfg & 0x8000)}')
v = xfer(bytes([addr, 21]), None)
if not v: raise SystemExit('no reply to version request (wrong port, baud, address or mode?)')
print('version:', v[:v.index(b"\x00")].decode(errors='replace').strip() if b'\x00' in v else v)
cfg = struct.unpack('>H', read(99, 4))[0]; show(cfg, 'current')
print('serial timeout: %.1f s' % (read(15, 3)[0] / 10))
print('main battery: %.1f V' % (struct.unpack('>H', read(24, 4))[0] / 10))
if args.set_mode or args.set_baud or args.set_addr:
    new = cfg
    if args.set_mode: new = (new & ~0x3) | ['rc', 'analog', 'simple', 'packet'].index(args.set_mode)
    if args.set_baud: new = (new & ~0xE0) | (BAUDS.index(args.set_baud) << 5)
    if args.set_addr: new = (new & ~0x700) | ((args.set_addr - 128) << 8)
    write(98, struct.pack('>H', new)); show(struct.unpack('>H', read(99, 4))[0], 'now')
if args.timeout is not None:
    write(14, bytes([args.timeout])); print('serial timeout now: %.1f s' % (read(15, 3)[0] / 10))
if args.save:
    write(94); print('saved to EEPROM')
