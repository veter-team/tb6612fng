LDFLAGS += -lrtdm

XENOCONFIG = xeno-config
CFLAGS += $(shell $(XENOCONFIG) --skin=native --cflags) $(MY_CFLAGS)
LDFLAGS += $(shell $(XENOCONFIG) --skin=native --ldflags) $(MY_LDFLAGS) -lnative

# This includes the library path of given Xenomai into the binary to make live
# easier for beginners if Xenomai's libs are not in any default search path.
LDFLAGS += -Xlinker -rpath -Xlinker $(shell $(XENOCONFIG) --libdir)


all: tb6612fng-ctl

tb6612fng-ctl: tb6612fng-ctl.o
	$(CC) tb6612fng-ctl.o $(LDFLAGS) -o tb6612fng-ctl

tb6612fng-ctl.o: tb6612fng-ctl.c ../interface.h
	$(CC) -c $(CFLAGS) tb6612fng-ctl.c

clean:
	rm -f tb6612fng-ctl.o tb6612fng-ctl

