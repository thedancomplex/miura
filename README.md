miura
=====

Control system for high DOF robots and actuators.  Controller talks to actuators over ethernet.  

RT On Raspberry PI
==================

Get the 3.8.13 kernel source from git:
CODE: SELECT ALL
	git clone https://github.com/raspberrypi/linux.git
	cd linux
	git checkout rpi-3.8.y

Get the toolchain from git:
CODE: SELECT ALL
	git clone https://github.com/raspberrypi/tools.git

Download the 3.8.13-rt11 patch from https://www.kernel.org/pub/linux/kernel ... ts/rt/3.8/
Patch the source:
CODE: SELECT ALL
	bzcat patch-3.8.13-rt11.patch.bz2 | patch -p1

Start with the stock .config (or get one from a running RasPi). Suppose you've got /proc/config.gz from a running raspi:
CODE: SELECT ALL
	zcat config.gz > .config

Set up your environment:
CODE: SELECT ALL
	export ARCH=arm
	export CROSS_COMPILE=/home/me/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-
	export INSTALL_MOD_PATH=/home/me/raspi-image

Configure the kernel:
CODE: SELECT ALL
	make gconfig

Enable PREEMPT-RT and any other RT features you want
Save your configuration
Build the kernel - add '-j n' where n is the number of cores available + 1 to speed it up:
CODE: SELECT ALL
	make bzImage
	make modules
	make modules_install

Take the stock Raspbian image and install it on the SD card as per normal.
Copy linux/arch/arm/boot/zImage to /boot/zImage-3.8.13-rt11 on the RPI.
Copy raspi-image/lib/modules/3.8.13-rt11+ to /lib/modules on the RPI taking care to preserve symlinks - use cp -rp.
On the RPI, edit /boot/config.txt. Comment out any line that starts with 'kernel=' and add this line:
CODE: SELECT ALL
	kernel=zImage-3.8.13-rt11

On the RPI, edit /boot/cmdline.txt. Add this option:
CODE: SELECT ALL
	sdhci_bcm2708.enable_llm=0

Done. Your RPi should boot a realtime-capable kernel.

TODOs:
	Debug the MMC driver so it works in RT with enable_llm=1.

Note that if you want to use ftrace on the RPI, you need to do this:
CODE: SELECT ALL
	echo dwc_otg_hcd_handle_fiq > /sys/kernel/debug/tracing/set_ftrace_filter

before you enable any tracer. Otherwise the USB driver will crash the kernel.

BTW here is the current performance on my RPI:
CODE: SELECT ALL
	root@raspberrypi:/home/pi/rt-tests# ./cyclictest -p95 -n -l 1000000
	# /dev/cpu_dma_latency set to 0us
	policy: fifo: loadavg: 0.45 0.42 0.32 1/143 4572

	T: 0 ( 4523) P:95 I:1000 C: 120110 Min:     20 Act:   44 Avg:   41 Max:  109
