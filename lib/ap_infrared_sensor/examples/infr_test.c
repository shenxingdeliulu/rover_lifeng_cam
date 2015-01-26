#include <stdio.h>
#include "ad_infr.h"
#include "scheduler.h"

int main()
{
	float distance;
	infr_init();
	scheduler_init();
	while(1)
	{
		if (read_infr(&distance) == 0)
		{
			fprintf(stdout, "the distance is %f\n", distance);
		}
		delay_ms(1000);
		//fprintf(stdout, "wat is %f\n", distance);
	}
	infr_close();
}
