#include <stdio.h>
#include <unistd.h>
#include "ap_gps.h"
//#include "ap_location.h"
#include "scheduler.h"

int main()
{
	scheduler_init();
	struct location cur;
	unsigned long timer = 0;
	get_ms(&timer);
	fprintf(stdout, "\ntime stamp is :%lu \n", timer);
	gps_init();
	while(1)
	{
		gps_parse();
		//ground_location(&cur);
		check_position();
		fprintf(stdout, "\nlast good update is  :%lu ms\n", last_good_update);
		delay_ms(1000);
		fprintf(stdout, "\ngps_quality is :%d \n", gps_quality());
		fprintf(stdout, "\nOperating mode is :%d \n", gps_op_mode());
		//fprintf(stdout,"\ntime:%d,%d,%d,%d,%d,%d\n", beiJingTime.year+1900, beiJingTime.mon+1,beiJingTime.day,beiJingTime.hour,beiJingTime.min,beiJingTime.sec);
        //fprintf(stdout, "\nlatitude:%f,longitude:%f\n",info.lat,info.lon);
        fprintf(stdout, "\nlatitude:%ld,longitude:%ld\n",gps_loc.lat,gps_loc.lng);
        //fprintf(stdout, "\nthe satellite being used:%d,the visible satellite:%d\n",info.satinfo.inuse,info.satinfo.inview);
        fprintf(stdout, "\nthe satellite being used:%d\n",num_sats());
        //fprintf(stdout, "\naltitude:%f m\n", info.elv);
        fprintf(stdout, "\naltitude:%ld cm\n", gps_loc.alt);
        //fprintf(stdout, "\nspeed:%f km/h\n", info.speed);
        fprintf(stdout, "\nspeed:%f m/s\n", ground_speed());
        fprintf(stdout, "\nspeed:%ld cm/s\n", ground_speed_cm());
        //fprintf(stdout, "\ndirection:%f degree\n", ground_speed());
        fprintf(stdout, "\ndirection:%ld centidegrees\n", ground_course_cd());
        get_ms(&timer);
		fprintf(stdout, "\ntime stamp is :%lums \n", timer);
		// get_us(&timer);
		// fprintf(stdout, "\ntime stamp is :%luus \n", timer);
		//fprintf(stdout, "\ndeclination:%f \n", info.declination);


	}
}