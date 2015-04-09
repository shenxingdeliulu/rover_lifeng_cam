#include <stdio.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>

#include "scheduler.h"
#include "my_timer.h"
#include "settings.h"
#include "communication.h"
#include "task.h"
#include "imu.h"
#include "ap_gps.h"
#include "ap_control.h"

int register_sig_handler();

void sigint_main_handler(int sig);


int main_done = 0;

int  rover_init()
{
	scheduler_init();
	scheduler_begin(timer_update);
	global_data_reset_param_defaults();
	register_sig_handler();
	return 0;

}
int main()
{

	rover_init();

	int res;
	pthread_t transfer_thread;
	pthread_t read_imu_thread;
	pthread_t read_gps_thread;
	pthread_t control_thread;

	void *thread_result;
	if (communication_init() == 0)
	{
		flag_communication_init = true;
		res = pthread_create(&transfer_thread, NULL, task_transfer, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:transfer failed:%s\n", strerror(errno));
		}
	}

	if (mpu9150_init(i2c_bus_imu, sample_rate, yaw_mix_factor) == 0)
	{
		flag_imu_init = true;
		res = pthread_create(&read_imu_thread, NULL, task_read_imu, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:read imu failed:%s\n", strerror(errno));
		}
	}

	if (gps_init() == 0)
	{
		flag_gps_init = true;
		res = pthread_create(&read_gps_thread, NULL, task_read_gps, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:read gps failed:%s\n", strerror(errno));
		}
	}

	if (control_init() == 0)
	{
		flag_control_init = true;
		res = pthread_create(&control_thread, NULL, task_control, 0);
		if (res != 0)
		{
			fprintf(stderr, "task:control failed:%s\n", strerror(errno));
		}
	}

	while(main_done == 0)
	{

	}


	//res = pthread_join(transfer_thread, &thread_result);
	return 0;
}

int register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_main_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		//exit(1);
		return -1;
	}
	return 0;
}

void sigint_main_handler(int sig)
{
	sigint_handler(sig);
	main_done = 1;


}

