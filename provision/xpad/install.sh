#!/bin/bash
# Build xpad.ko for the running kernel and install it under /lib/modules/<kernel>/updates/, which
# depmod searches before the stock module. Re-run after every kernel (nvidia-l4t-kernel) update.
#   ./install.sh            build + install (takes effect at the next xpad load / reboot)
#   ./install.sh --reload   also swap the running module now (the joystick vanishes for a second;
#                           joy_node is respawned by robot_disco's launch if it dies)
set -euo pipefail
cd "$(dirname "$0")"
KVER=$(uname -r)
[ -e "/lib/modules/$KVER/build" ] || { echo "no kernel headers for $KVER (apt install nvidia-l4t-kernel-headers)"; exit 1; }
make -s clean >/dev/null 2>&1 || true
make -s
modinfo ./xpad.ko | grep -E "^vermagic"
modinfo ./xpad.ko | grep -q "v2DC8p\*.*icFFisc5Dip01" && echo "match table: 8BitDo (2dc8) X-input interfaces present"
sudo install -D -m 644 xpad.ko "/lib/modules/$KVER/updates/xpad.ko"
sudo depmod -a
echo "modprobe now resolves to: $(modinfo -n xpad)"
if [ "${1:-}" = "--reload" ]; then
  sudo rmmod xpad 2>/dev/null || true
  sudo modprobe xpad
  sleep 1
  echo "loaded: $(cat /sys/module/xpad/srcversion 2>/dev/null) leds: $(ls -d /sys/class/leds/xpad* 2>/dev/null | tr '\n' ' ')"
fi
