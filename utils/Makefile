objects = tb6612fng-ctl twiddle

BINDIR := /usr/bin

# Check if xeno-config in the path
XENOCONFIG = $(shell which xeno-config)

ifeq ($(XENOCONFIG),)
    # No xeno-config, hardcoded for cross compile
    XENO_LIBS = -lpthread -lrt -lnative -lrtdm -lxenomai
    CFLAGS += $(XENO_INCDIR) -D_GNU_SOURCE -D_REENTRANT -D__XENO__
    LDFLAGS += $(XENO_LIBS)
else
    # defining flags with xeno-config
    CFLAGS += $(shell $(XENOCONFIG) --skin=native --cflags) 
    LDFLAGS += $(shell $(XENOCONFIG) --skin=native --ldflags) -lnative -lrtdm
    LDFLAGS += -Xlinker -rpath -Xlinker $(shell $(XENOCONFIG) --libdir)
endif

.PHONY : all
all : $(objects)

tb6612fng-ctl: tb6612fng-ctl.o
	$(CC) $< $(LDFLAGS) -o $@

twiddle: twiddle.o
	$(CC) $< $(LDFLAGS) -o $@

tb6612fng-ctl.o: tb6612fng-ctl.c ../interface.h
	$(CC) -c $(CFLAGS) tb6612fng-ctl.c

twiddle.o: twiddle.c ../pid-params.h
	$(CC) -c $(CFLAGS) twiddle.c

.PHONY : clean
clean:
	rm -f *.o ${objects}

