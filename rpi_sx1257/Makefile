CROSS_COMPILE?=arm-linux-gnueabihf-
CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -std=c99 -D_XOPEN_SOURCE=500 -O2 $(ARM_OPTIONS)
LDFLAGS+=-lpthread -lm -lwiringPi

all: clean main gpio-interrupt

main: main.o
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

gpio-interrupt: gpio-interrupt.o
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *.o main gpio-interrupt