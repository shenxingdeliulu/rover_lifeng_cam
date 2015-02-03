#
# Makefile for eMPL Linux userland implementation
#

CC = arm-linux-gcc
CFLAGS = -Wall -fsingle-precision-constant

# add -DI2C_DEBUG -DMPU9150_DEBUG for debugging 
DEFS = -DEMPL_TARGET_LINUX -DMPU9150 -DAK8975_SECONDARY 

EMPLDIR = ../eMPL
HALDIR = ../../ap_hal_linux
MPUDIR = ../mpu9150

OBJS = inv_mpu.o \
       inv_mpu_dmp_motion_driver.o \
       i2c_driver.o \
       scheduler.o \
       mpu9150.o \
       quaternion.o \
       vector3d.o

INCS = -I ../ -I $(EMPLDIR) -I $(HALDIR) -I $(MPUDIR) 

all : imu imucal


imu : $(OBJS) imu.o
	$(CC) $(CFLAGS) $(OBJS) imu.o -lm -lrt -o imu

imucal : $(OBJS) imucal.o
	$(CC) $(CFLAGS) $(OBJS) imucal.o -lm -lrt -o imucal

	
imu.o : imu.c
	$(CC) $(CFLAGS) $(INCS)  $(DEFS)  -c imu.c
	
imucal.o : imucal.c
	$(CC) $(CFLAGS) $(INCS) -I $(MPUDIR) $(DEFS) -c imucal.c

mpu9150.o : $(MPUDIR)/mpu9150.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $(MPUDIR)/mpu9150.c

quaternion.o : $(MPUDIR)/quaternion.c
	$(CC) $(CFLAGS) $(DEFS) -c $(MPUDIR)/quaternion.c

vector3d.o : $(MPUDIR)/vector3d.c
	$(CC) $(CFLAGS) $(DEFS) -c $(MPUDIR)/vector3d.c

i2c_driver.o : $(HALDIR)/i2c_driver.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $(HALDIR)/i2c_driver.c

scheduler.o : $(HALDIR)/scheduler.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $(HALDIR)/scheduler.c

inv_mpu_dmp_motion_driver.o : $(EMPLDIR)/inv_mpu_dmp_motion_driver.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $(EMPLDIR)/inv_mpu_dmp_motion_driver.c

inv_mpu.o : $(EMPLDIR)/inv_mpu.c
	$(CC) $(CFLAGS) $(DEFS) $(INCS) -c $ -c $(EMPLDIR)/inv_mpu.c

clean:
	rm -f *.o imu imucal