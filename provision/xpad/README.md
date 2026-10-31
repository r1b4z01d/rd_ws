# xpad with LED + rumble output for the Tegra kernel

The 8BitDo Ultimate 2C's 2.4 GHz receiver (USB 2dc8:310a, Xbox-360-class interface) does not
send a single input report until the host has sent it the Xbox 360 LED command (`01 03 0x`).
JetPack's kernel builds xpad with `CONFIG_JOYSTICK_XPAD_LEDS` and `CONFIG_JOYSTICK_XPAD_FF`
off, so the stock module never sends any output packet: the receiver enumerates, xpad binds, a
joystick appears and nothing ever moves (measured 2026-10-31 with a kernel xhci trace: zero
interrupt-IN completions on all three interfaces while the pad was used; after one LED command
sent over usbfs the receiver answered with `00 14 …` reports at once).

This directory builds the same driver out of tree with both options on and with the receiver in
the device table (the upstream 6.11 entries), so it binds by itself, names the pad
"8BitDo Ultimate 2C Wireless Controller", sends the LED command at probe and supports rumble.

## Files

- `xpad.c` — `drivers/input/joystick/xpad.c` from the stable `linux-5.15.y` branch
  (github.com/gregkh/linux, head 700e62ea909a 2026-10-30), GPL-2.0, with three lines added:
  the two `0x2dc8` device-table entries (`0x3106`, `0x310a`) and
  `XPAD_XBOX360_VENDOR(0x2dc8)` in the match table.
- `Makefile` — `ccflags-y := -DCONFIG_JOYSTICK_XPAD_FF=1 -DCONFIG_JOYSTICK_XPAD_LEDS=1`.
- `install.sh` — builds against `/lib/modules/$(uname -r)/build` (package
  `nvidia-l4t-kernel-headers`), installs to `/lib/modules/<kernel>/updates/xpad.ko` (depmod
  prefers `updates/` over the stock module), `--reload` swaps the running module.
- `tools/xpad_poke.py` — the diagnostic that found this: detaches xpad from interface 0, sends
  the LED command through usbfs and prints what the input endpoint returns, then re-attaches the
  driver. `sudo python3 tools/xpad_poke.py /dev/bus/usb/<bus>/<dev> 10`.

## On the robot

```
rsync -a rd_ws/provision/xpad/ RD:rd_ws/provision/xpad/
ssh RD '~/rd_ws/provision/xpad/install.sh --reload'
```

Re-run `install.sh` after every kernel update (`nvidia-l4t-kernel`): the module in `updates/`
is tied to one kernel version and is simply absent for a new one, which falls back to the stock
module (pad silent again). Module signing is not enforced on the Jetson (`sig_enforce=N`), so
the unsigned build loads (it taints the kernel, harmless).

Check after loading: `/sys/class/leds/xpad0` exists, `dmesg` shows
`input: 8BitDo Ultimate 2C Wireless Controller`, and `ros2 topic echo /joy` changes when the
pad is used. `config/joystick.yaml` must carry that device name (the stock module called it
"Generic X-Box pad"). The udev rule in `provision/udev/99-robot-disco.rules` that writes the
ID to xpad's `new_id` is kept as a fallback for the stock module; with this build it is a no-op.
