#include "ad_driver.h"
#include "ad_infr.h"
#include "scheduler.h"

//parameter
float infrared_m = 2.67e-5;
float infrared_b = -4.08e-3;
int   infrared_k = 4;

uint32_t last_good_time_infr = 0;
bool flag_infr_init = false;

int infr_init()
{
    if (ad_init() == 0)
    {
        
        flag_infr_init = true;
        return 0;
    }
    else 
    {
        return -1;
    }
}

void infr_close()
{
    if (flag_infr_init)
    {
        ad_close();
    }
    
}

int  read_infr(float *distance_infr)
{
    
    int value_ad;
    uint32_t now;
    get_ms(&now);
    if (!flag_infr_init)
    {
        return -1;
    }
    if (read_ad_value(&value_ad) == 0)
    {
        *distance_infr = 1 / (value_ad * infrared_m + infrared_b) - infrared_k;
        return 0;
    }   
    else 
    {
        return -1;
    }         

}
