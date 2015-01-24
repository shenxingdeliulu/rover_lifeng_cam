
#webcam_server webcam_shower
#CC=gcc
#LD=ld
CC=arm-linux-gcc
LD=arm-linux-ld
CFLAGS=	-c -Wall -std=gnu99
#CXXFLAGS=	-c -g -O0 -fPIC -I/opt/webcam/ffmpeg/include -I/usr/include -L/opt/webcam/ffmpeg/lib
OBJS=	 gps_test.o  ../ap_gps.o ../../ap_hal_linux/uart_driver.o  \
			../../ap_hal_linux/scheduler.o ../../ap_math/ap_location.o \
			../../ap_math/ap_math.o ../../mesch12b/meschach.a 

LIBS=	   -Llib -lnmea -lrt -lm 

INCS = -I../../nmealib/include  -I../../ap_hal_linux -I../  \
			-L../../nmealib/lib -I../../ap_math -I../../mesch12b \
		
.PHONY: all all-before gps_test

all: all-before gps_test	

#all-before:
#	cd ../../nmealib && $(MAKE)

%.o: %.c 
	$(CC)  $(CFLAGS) $(INCS) $< -o $@  

../../mesch12b/meschach.a:
	cd ../../mesch12b/ && $(MAKE) 

gps_test: $(OBJS)	
	$(CC) $(INCS)  $^ $(LIBS)    -o $@

clean:
	rm -f $(OBJS)
	rm -f gps_test
	