default: all

CFLAGS := -I./include -g --std=gnu99
CC := gcc

BINARIES := miura-daemon
all : $(BINARIES)

LIBS := -lach -lrt -lm -lc
LIBSPI := -lbcm2835

miura-daemon: src/miura-daemon.o
	$(CC) -o $@ $< $(LIBS) $(LIBSPI)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o
