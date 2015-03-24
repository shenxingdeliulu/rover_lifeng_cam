#include <stdio.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>


#include "scheduler.h"
#include "my_timer.h"
#include "settings.h"
#include "communication.h"
#include "task.h"

int main()
{
	scheduler_init();
	scheduler_begin(timer_update);
	global_data_reset_param_defaults();
	communication_init();

	int res;
	pthread_t transfer_thread;
	void *thread_result;

	res = pthread_create(&transfer_thread, NULL, thread_transfer, 0);
	if (res != 0)
	{
		fprintf(stderr, "thread transfer failed:%s\n", strerror(errno)); 
	}

	res = pthread_join(transfer_thread, &thread_result);
	return 0;
}
