#include "communication.h"
#include "settings.h"
#include "my_timer.h"

void *thread_transfer()
{
	while(1)
	{
		if (send_system_state_now)
		{
			if (global_data.param[PARAM_SYSTEM_SEND_STATE])
			{
				communication_system_state_send();
			}
			send_system_state_now = false;
		}

		if (send_params_now)
		{
			communication_parameter_send();
			send_params_now = false;
		}

		if (send_gps_now)
		{
			send_gps_now = false;
		}

		if (receive_now)
		{
			communication_receive();
			receive_now = false;
		}
	}
}
