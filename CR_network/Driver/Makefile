ifeq ($(KERNELRELEASE),)

KERNELDIR ?= /usr/src/linux-headers-`uname -r`
PWD	:= $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) LDDINC=$(PWD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions [mM]odule*

else

obj-m	:= CR_net_device.o

endif
