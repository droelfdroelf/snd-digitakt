snd-digitakt-objs := digitakt.o
obj-m+=snd-digitakt.o

kernel_build := /lib/modules/$(shell uname -r)/build
mod_dir := $(shell pwd)
mod_make := make -C $(kernel_build) M=$(mod_dir)

all:
	$(mod_make) modules

clean:
	$(mod_make) clean

install: all
	$(mod_make) modules_install
	depmod


kernel_version := $(shell dpkg-query --show --showformat='$${Version}' \
	'raspberrypi-kernel-headers*' | sed -e 's/~stretch//')
kernel_repo := https://github.com/raspberrypi/linux
kernel_branch := $(kernel_repo)/raw/raspberrypi-kernel_$(kernel_version)

from-kernel:
	mkdir -p $@ 2>&1 || true

from-kernel/%.h: from-kernel
	curl -o $@ -L $(kernel_branch)/sound/usb/$*.h

extra-kernel-headers: from-kernel/midi.h from-kernel/usbaudio.h

digitakt.o: extra-kernel-headers

