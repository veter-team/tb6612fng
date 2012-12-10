obj-m := tb6612fng.o

tb6612fng-objs := tb6612fng-drv.o pwm-task-proc.o divconst.o gpio-manip.o pid-controller.o

# Default to sources of currently running kernel
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

# Module sources
SRC := $(shell pwd)

# Include xenomai headers
EXTRA_CFLAGS += -I$(KERNEL_SRC)/include/xenomai -I$(KERNEL_SRC)/include/xenomai/posix

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
