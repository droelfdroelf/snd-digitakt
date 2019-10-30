# snd-digitakt
ALSA driver for the Elektron Digitakt

## Build as an installable kernel module on a running system

The driver can now be built on your system without pulling an entire kernel
tree. However, the build requires two files from the kernel tree that are
not normally exported into the kernel headers package. The build will
attempt to grab them from github. If that doesn't work right, you may need
to find them yourself. See the Makefile.

### Build

    cd sound/usb/misc
	make

At this point you could simply manually install the module into your kernel:

    insmod snd-digitakt.ko

But, you probably want to install it...

### Install

    sudo make install

### Load it

    sudo modprobe snd-digitakt


### Run it

1. Connect your Digitakt
2. Put it in Overbridge mode
   * Settings (gear button) > System > USB Config
   * Select Overbridge, press Yes
   * Select Int to Main, set to Off

Now you should be able to see the device in ALSA's utitlies:

    aplay -l
	arecord -l

If you want to use Jack with it, try something like this:

    /usr/bin/jackd -R -P 75 -d alsa -d hw:Digitakt -r 48000 -n 2 -p 256 -s &


### Test with SuperCollider

Run `sclang`, then type the following to it:

    s.options.numInputBusChannels = 12
	s.boot
	{ SoundIn.ar([0, 1]_; }.play   // plays master bus out to Digitakt's outs
	CmdPeriod.run                  // stop that
	{ SoundIn.ar([2, 3]_; }.play   // plays tracks 1 and 2 to Digitakt's outs
	CmdPeriod.run                  // stop that
	{ SoundIn.ar([10, 11]_; }.play // plays Digitakt's ins to Digitakt's outs 
	CmdPeriod.run                  // stop that

## Previous build instructions, if you need them

Build instructions for Raspberry (tnx mzero!)
https://gist.github.com/mzero/b8d772cd78867a3215fcd7a0e42add51
