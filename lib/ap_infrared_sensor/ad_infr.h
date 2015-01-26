#ifndef AD_INFR_H
#define AD_INFR_H
#include <stdbool.h>
#include <stdint.h>

extern uint32_t last_good_time_infr ;
extern bool flag_infr_init;

//parameter
extern float infrared_m;
extern float infrared_b;
extern int   infrared_k;

int infr_init();
void infr_close();
int  read_infr(float *distance_infr);

#endif