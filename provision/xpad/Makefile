# Out-of-tree build of the kernel's xpad driver with the two Kconfig options the Tegra kernel
# leaves off (LED + rumble output packets) and the 8BitDo 2.4 GHz receiver in the device table.
obj-m := xpad.o
ccflags-y := -DCONFIG_JOYSTICK_XPAD_FF=1 -DCONFIG_JOYSTICK_XPAD_LEDS=1
KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
