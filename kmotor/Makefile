obj-m := mz-kmotor.o
mz-kmotor-objs := kmotor.o ../fixed/fixed.o
ccflags-y += -g -DDEBUG

# conditional if not on COE systems 
#  - check if $HOST looks identical after replacing coe.neu.edu with nothing
ifeq ($(subst coe.neu.edu,,$(shell hostname --domain)),$(shell hostname --domain))
# not COE must be local install o
KERNEL_SRC ?= /home/build/work/linux
CROSS_COMPILE ?= /usr/bin/arm-linux-gnueabihf-
ZEDBOARD_IP?=1.2.2.1
else 
# COE
KERNEL_SRC ?= /ECEnet/Apps1/esl/linux-4.9-zed
CROSS_COMPILE ?= /ECEnet/Apps1/esl/gcc-7.2.1-arm/bin/arm-linux-gnueabihf-
ZEDBOARD_IP?=qemu
endif
ARCH ?= arm
SRC := $(shell pwd)
UPLOAD_PATH ?= /home/root/

MODULE_OBJ:=$(obj-m:.o=.ko)

all: modules

modules:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(SRC) modules_install

modules_upload: modules
	scp $(MODULE_OBJ) root@$(ZEDBOARD_IP):$(UPLOAD_PATH)

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
