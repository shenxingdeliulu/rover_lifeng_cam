#include "scheduler.h"
#include "my_timer.h"

#define TIMER_DATA_LENGTH 15

bool send_system_state_now = true;
bool receive_now = true;
bool send_params_now = true;
bool send_gps_now = true;
bool send_imu_now = true;
bool read_imu_now = true;
bool read_infr_now = true;
bool read_gps_now = true;
bool update_current_mode = true;
bool set_servos_now = true;
bool navigate = true;

enum 
{
	TIMER_SYSTEM_STATE,
	NTIMERS 
} timer_state;

struct 
{
	int timer[NTIMERS];
	char timer_name[NTIMERS][TIMER_DATA_LENGTH];

} timer_data;


void timer_data_defaluts()
{

}
void timer_update(union sigval v)
{
	unsigned i = 0;
	for ( i = 0; i < NTIMERS; i++)
	{
		if (timer[i] > 0)
			timer[i]--;
	}
	if (timer[TIMER_SYSTEM_STATE] == 0)
	{
		send_system_state_now = true;
		timer[TIMER_SYSTEM_STATE] = SYSTEM_STATE_COUNT;
	}
	if (timer[TIMER_RECEIVE] == 0)
	{
		receive_now = true;
		timer[TIMER_RECEIVE] = RECEIVE_COUNT;
	}
	if (timer[TIMER_PARAMS] == 0)
	{
		send_params_now = true;
		timer[TIMER_PARAMS] =PARAMS_COUNT;
	}
}