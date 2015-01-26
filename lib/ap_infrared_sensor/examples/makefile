
#webcam_server webcam_shower
#CC=gcc
#LD=ld
CC=arm-linux-gcc
LD=arm-linux-ld
CFLAGS=	-c -Wall -std=gnu99 

#DEFS = -DGPS_DEBUG
DEFS = -DAD_DEBUG

INFRDIR = ..
HALDIR = ../../ap_hal_linux
#CXXFLAGS=	-c -g -O0 -fPIC -I/opt/webcam/ffmpeg/include -I/usr/include -L/opt/webcam/ffmpeg/lib
OBJS=	 infr_test.o \
			$(INFRDIR)/ad_infr.o \
			$(HALDIR)/scheduler.o \
 			$(HALDIR)/ad_driver.o

#LIBS=	   -Llib -lnmea -lrt -lm 
LIBS = -lrt
INCS =  -I $(HALDIR) -I $(INFRDIR)

.PHONY: all  infr_test

all:  infr_test	

#all-before:
#	cd ../../nmealib && $(MAKE)

%.o: %.c 
	$(CC)  $(CFLAGS) $(INCS) $<  $(DEFS) -o $@  
 

infr_test: $(OBJS)	
	$(CC) $(INCS)  $^ $(LIBS)    -o $@

clean:
	rm -f $(OBJS)
	rm -f infr_test
	