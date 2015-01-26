#ifndef AD_DRIVER_H
#define AD_DRIVER_H

int ad_init();
void ad_close();
int read_ad_vol(float *ad_vol);
int read_ad_value(int *ad_value);

#endif