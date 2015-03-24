all: test

CC = arm-linux-gcc
CFLAGS = -Wall -std=gnu99 -c

SRCDIR = ./src
MAVLINKDIR = ./lib/mavlink
MAVLINKDIR2 = ./lib/mavlink/common
HALDIR = ./lib/ap_hal_linux

OBJS = $(SRCDIR)/communication.o \
		$(SRCDIR)/settings.o \
		$(SRCDIR)/my_timer.o \
		$(SRCDIR)/communication.o \
		$(SRCDIR)/task.o \
		$(SRCDIR)/rover.o \
		$(HALDIR)/udp_driver.o \
		$(HALDIR)/scheduler.o

LIBS = -lpthread -lrt

INCS = -I $(MAVLINKDIR) -I $(MAVLINKDIR2) -I $(SRCDIR)  -I $(HALDIR)


%.o: %.c
	$(CC) $(CFLAGS) $(INCS) $< -o $@

test: $(OBJS)
	$(CC) $(INCS) $^ $(LIBS) -o $@

clean:
	rm -f $(OBJS)
