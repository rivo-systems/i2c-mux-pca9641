MODULE := i2c-mux-pca9641

ifeq ($(KERNELRELEASE),)

	KERNELDIR ?= /lib/modules/$$(uname -r)/build
	PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
	$(MAKE) copy

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

copy:
	sudo cp $(MODULE).ko /lib/modules/$$(uname -r)/kernel/drivers/i2c/muxes; sudo modprobe -r $(MODULE); sudo modprobe $(MODULE)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

.PHONY: modules modules_install clean

else
	obj-m := $(MODULE).o
endif
