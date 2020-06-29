# KTMicro's KT0913 Linux driver
This driver exposes a V4L2 tuner interface for the KT0913 chip, which is used over a standard I2C bus.
The KT0913 chip is an inexpensive all-in-one AM/FM tuner with minimum components required for its usage.

## How to compile this driver
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

## How to use this driver
Since the V4L2 interface is standard, you can use any application that knows how to interface with a tuner.
I suggest using `radio`, a ncurses-based tuner app, which comes from the [xawtv](https://linuxtv.org/wiki/index.php/Xawtv#Associated_Utilities) package. Usually `sudo apt install -y radio` does it under a Ubuntu/Debian distro.
You can use any other app, like the ones described on [LinuxTV's wiki](https://linuxtv.org/wiki/index.php/Radio_Listening_Software).
