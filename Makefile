# This file comes from
# https://github.com/allanbian1017/i2c-ch341-usb/blob/master/Makefile
PWD         := $(shell pwd)
KVERSION    := $(shell uname -r)
KERNEL_DIR   = /lib/modules/$(KVERSION)
KERNEL_INST  = $(KERNEL_DIR)/kernel/drivers/media/radio

MODULE_NAME  = radio-kt0913
obj-m       := $(MODULE_NAME).o

$(MODULE_NAME).ko:
	make -C $(KERNEL_DIR)/build M=$(PWD) modules

all: $(MODULE_NAME).ko
	modinfo $(MODULE_NAME).ko

clean:
	make -C $(KERNEL_DIR)/build M=$(PWD) clean

rpi4-ktoverlay.dtbo:
	dtc -@ -Hepapr -I dts -O dtb -o rpi4-ktoverlay.dtbo fragment.dts

rpi4-install: $(MODULE_NAME).ko rpi4-ktoverlay.dtbo
	cp $(MODULE_NAME).ko $(KERNEL_INST)/
	depmod
	modprobe $(MODULE_NAME) kt0913_use_campus_band=1
	dtoverlay rpi4-ktoverlay.dtbo
	sleep 1
	v4l2-ctl -d /dev/radio0 --all
	v4l2-ctl -d /dev/radio0 -f 97.9

rpi4-clean: clean
	-dtoverlay -r rpi4-ktoverlay
	-rmmod $(MODULE_NAME)
	-rm $(KERNEL_INST)/$(MODULE_NAME).ko
	-depmod
