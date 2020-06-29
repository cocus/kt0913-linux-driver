# KTMicro's KT0913 Linux driver
This driver exposes a V4L2 tuner interface for the KT0913 chip, which is used over a standard I2C bus.
The KT0913 chip is an inexpensive all-in-one AM/FM tuner with minimum components required for its usage.

## How to use this driver
You'll need support for your kernel, which consist of a gcc toolchain and the driver kernel headers. On a Ubuntu/Debian distro, run:
`sudo apt install -y build-essential linux-headers-$(uname -r)`

### Generic target
To make the kernel module and insert it on the kernel, just run:
```
make
sudo insmod radio-kt0913.ko
```
To remove it, just run:
```
sudo rmmod radio-kt0913
```

### RaspberryPi4
If you happen to be using this chip on a RaspberryPi4 (i.e. i2c overlay enabled, and the KT0913 connected to the I2C pins), you can make usage of the additional targets of the Makefile, as follows:
```
make
sudo make rpi4-install
```
To remove it:
```
sudo make rpi4-clean
```
