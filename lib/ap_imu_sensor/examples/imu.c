
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>
#include <stdbool.h>
#include "mpu9150.h"
#include "i2c_driver.h"
#include "local_defaults.h"
#include "scheduler.h"

int set_cal(int mag, char *cal_file);
void read_loop(unsigned int sample_rate);
void print_fused_euler_angles(mpudata_t *mpu);
void print_fused_quaternions(mpudata_t *mpu);
void print_calibrated_accel(mpudata_t *mpu);
void print_calibrated_mag(mpudata_t *mpu);
void register_sig_handler();
void sigint_handler(int sig);

int done = 0;

int timer_read_data = 20;
int timer_print_mpu = 1000;
bool flag_read_imu = false;
bool flag_print_mpu = false;

void timer_update(union sigval v)
{
	timer_read_data--;
	timer_print_mpu--;
	if (timer_read_data <= 0)
	{
		flag_read_imu = true;
		timer_read_data = 20; //20ms
	}
	if (timer_print_mpu <=0)
	{
		flag_print_mpu = true;
		timer_print_mpu = 1000; //1s
	}
	

}

void usage(char *argv_0)
{
	printf("\nUsage: %s [options]\n", argv_0);
	printf("  -b <i2c-bus>          The I2C bus number where the IMU is. The default is 1 to use /dev/i2c-1.\n");
	printf("  -s <sample-rate>      The IMU sample rate in Hz. Range 2-50, default 10.\n");
	printf("  -y <yaw-mix-factor>   Effect of mag yaw on fused yaw data.\n");
	printf("                           0 = gyro only\n");
	printf("                           1 = mag only\n");
	printf("                           > 1 scaled mag adjustment of gyro data\n");
	printf("                           The default is 4.\n");
	printf("  -a <accelcal file>    Path to accelerometer calibration file. Default is ./accelcal.txt\n");
	printf("  -m <magcal file>      Path to mag calibration file. Default is ./magcal.txt\n");
	printf("  -v                    Verbose messages\n");
	printf("  -h                    Show this help\n");

	printf("\nExample: %s -b3 -s20 -y10\n\n", argv_0);
	
	exit(1);
}

int main(int argc, char **argv )
{
	mpudata_t mpu;
	uint32_t now;
	int opt;
	int verbose = 0;
	int len = 0;
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
	//int sample_rate = 100;
	int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
	char *mag_cal_file = NULL;
	char *accel_cal_file = NULL;
	done =0;
	while ( (opt = getopt(argc, argv, "b:s:y:a:m:vh")) != -1)
	{
		switch (opt)
		{
			case 'b':
				i2c_bus = strtoul(optarg, NULL, 0);
				if (errno == EINVAL)
				{
					usage(argv[0]);
				}
				if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS)
				{
					usage(argv[0]);
				}
				break;
			case 's':
				sample_rate = strtoul(optarg, NULL, 0);
				if (errno == EINVAL)
				{
					usage(argv[0]);
				}
				if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE)
				{
					usage(argv[0]);
				}
				break;
			case 'a':
				len = 1 + strlen(optarg);
				accel_cal_file = (char *)malloc(len);
				if (!accel_cal_file)
				{
					fprintf(stderr, "malloc err: %s\n", strerror(errno));
				}
				strcpy(accel_cal_file, optarg);
				break;
			case 'm':
				len = 1 + strlen(optarg);
				mag_cal_file = (char *)malloc(len);
				if (!mag_cal_file)
				{
					fprintf(stderr, "malloc err : %s\n", strerror(errno));
				}
				strcpy(mag_cal_file, optarg);
				fprintf(stdout, "mag_cal_file is %s\n", mag_cal_file);
				break;
			case 'y':
				yaw_mix_factor = strtoul(optarg, NULL, 0);
				if (errno == EINVAL)
				{
					usage(argv[0]);
				}
				if (yaw_mix_factor < 0 || yaw_mix_factor > 100)
				{
					usage(argv[0]);
				}
				break;
			case 'v':
				verbose = 1;
				break;
			case 'h':
			default:
				usage(argv[0]);
				break;
		}
	}
	scheduler_init();
	scheduler_begin(timer_update);
	register_sig_handler();
	mpu9150_set_debug(verbose);
	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor) == -1)
	{
		exit(1);
	}
	set_cal(0, accel_cal_file);
	set_cal(1, mag_cal_file);
	if (accel_cal_file)
	{
		free(accel_cal_file);
	}
	if (mag_cal_file)
	{
		free(mag_cal_file);
	}
	while(done == 0)
	{
		if (flag_read_imu)
		{
			//get_ms(&now);
			//fprintf(stdout, "time is %lu\n", (long unsigned int)now);
			flag_read_imu = false;
			if (mpu9150_read(&mpu) == 0)
			{
				//print_fused_euler_angles(&mpu);
				//fprintf(stdout, "read mpu9150 dmpTimestamp is %lu\n", mpu.dmpTimestamp);
				//fprintf(stdout, "read mpu9150 magTimestamp is %lu\n", mpu.magTimestamp);
			}
		}
		if (flag_print_mpu)
		{
			get_ms(&now);
			fprintf(stdout, "time is %lu\n", (long unsigned int)now);
			print_fused_euler_angles(&mpu);
			fprintf(stdout, "\n");
			flag_print_mpu = false;

		}
	}
	mpu9150_exit();
	return 0;
}

void print_fused_euler_angles(mpudata_t *mpu)
{
	printf("\rX: %0.2f Y: %0.2f Z: %0.2f        ",
			mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE);

	fflush(stdout);
}

void print_fused_quaternions(mpudata_t *mpu)
{
	printf("\rW: %0.2f X: %0.2f Y: %0.2f Z: %0.2f        ",
			mpu->fusedQuat[QUAT_W],
			mpu->fusedQuat[QUAT_X],
			mpu->fusedQuat[QUAT_Y],
			mpu->fusedQuat[QUAT_Z]);

	fflush(stdout);
}

void print_calibrated_accel(mpudata_t *mpu)
{
	printf("\rX: %05d Y: %05d Z: %05d        ",
			mpu->calibratedAccel[VEC3_X], 
			mpu->calibratedAccel[VEC3_Y], 
			mpu->calibratedAccel[VEC3_Z]);

	fflush(stdout);
}

void print_calibrated_mag(mpudata_t *mpu)
{
	printf("\rX: %03d Y: %03d Z: %03d        ",
			mpu->calibratedMag[VEC3_X], 
			mpu->calibratedMag[VEC3_Y], 
			mpu->calibratedMag[VEC3_Z]);

	fflush(stdout);
}

int set_cal(int mag, char *cal_file)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];
	caldata_t cal;

	if (cal_file) {
		f = fopen(cal_file, "r");
		
		if (!f) {
			perror("open(<cal-file>)");
			return -1;
		}
	}
	else {
		if (mag) {
			f = fopen("./magcal.txt", "r");
		
			if (!f) {
				printf("Default magcal.txt not found\n");
				return 0;
			}
		}
		else {
			f = fopen("./accelcal.txt", "r");
		
			if (!f) {
				printf("Default accelcal.txt not found\n");
				return 0;
			}
		}		
	}

	memset(buff, 0, sizeof(buff));
	
	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	cal.offset[0] = (short)((val[0] + val[1]) / 2);
	cal.offset[1] = (short)((val[2] + val[3]) / 2);
	cal.offset[2] = (short)((val[4] + val[5]) / 2);

	cal.range[0] = (short)(val[1] - cal.offset[0]);
	cal.range[1] = (short)(val[3] - cal.offset[1]);
	cal.range[2] = (short)(val[5] - cal.offset[2]);
	
	if (mag) 
		mpu9150_set_mag_cal(&cal);
	else 
		mpu9150_set_accel_cal(&cal);

	return 0;
}

void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	} 
}

void sigint_handler(int sig)
{
	done = 1;
}
