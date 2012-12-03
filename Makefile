obj-m += tb6612fng.o
tb6612fng-objs := tb6612fng-drv.o pwm-task-proc.o divconst.o gpio-manip.o

EXTRA_CFLAGS += -I/usr/include/xenomai/

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
