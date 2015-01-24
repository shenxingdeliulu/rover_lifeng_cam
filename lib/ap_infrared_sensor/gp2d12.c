#include "gp2d12.h"

#define ADC_SET_CHANNEL         0xc000fa01
#define ADC_SET_ADCTSC          0xc000fa02

/*
1) cd Linux-2.6.38
2) make menuconfig
3) enter Device Drivers--->Input device support  --->Touchscreens
4) unselect "S3C touchscreen driver for Mini6410"
5) make
*/

#define CHANNELCOUNT 6

#define VALUE_SIZE   25


float distance_infrared = 0;

char buffer_ad[30] ;
int value_ad = 0;

int channels[CHANNELCOUNT] = {0,1,4,5,6,7}; //for 6410
//int channels[CHANNELCOUNT] = {0,1,6,7,8,9}; //for 210
// int channels[CHANNELCOUNT] = {2,3,4,5};
int channel_ad,len;

char outbuff[VALUE_SIZE];

pthread_mutex_t lock_read_gp2d12 = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ready_read_gp2d12 = PTHREAD_COND_INITIALIZER;

char flag_read_gp2d12 = 0;

int fd_ir;
int ir_init()
{

    fd_ir= open("/dev/adc", 0);
    if (fd_ir < 0) {
        perror("open ADC device:");
        return -1;
    }

    //char output[255];
    channel_ad = channels[0];
    if (ioctl(fd_ir, ADC_SET_CHANNEL, channel_ad) < 0) {
        perror("Can't set channel_ad for /dev/adc!");
        close(fd_ir);
        return -1;
    }
    return 0;

}
void  *read_ir()
{

	while (1) {

        pthread_mutex_lock(&lock_read_gp2d12);
        while (flag_read_gp2d12 == 0){
            pthread_cond_wait(&ready_read_gp2d12, &lock_read_gp2d12);
        }
        pthread_mutex_unlock(&lock_read_gp2d12);
        //puts("\033[2J");
        //output[0] = 0;
        //for (i=0; i<CHANNELCOUNT; i++) {


           // char buffer[30];
            len = read(fd_ir, buffer_ad, sizeof buffer_ad -1);
            if (len > 0) {
                buffer_ad[len] = '\0';
                //int value = -1;
                sscanf(buffer_ad, "%d", &value_ad);
                memset(outbuff, 0, VALUE_SIZE);
            //    sprintf(outbuff, "\nAIN%d %d\n", channel_ad, value_ad);
                printf("%s",outbuff);

                distance_infrared = 1 / ((float)value_ad * global_data.param[PARAM_INFRARED_M] + global_data.param[PARAM_INFRARED_B]) - global_data.param[PARAM_INFRARED_K];
                //char outbuff[255];
                memset(outbuff, 0, VALUE_SIZE);
             //   sprintf(outbuff, "\nAIN%d %f\n", channel_ad, distance_infrared);
                //strcat(output, outbuff);
            } else {
                //perror("read ADC device:");
                fprintf(stderr, "\nread ADC device err:%s\n", strerror(errno));
                close(fd_ir);
                return 1;
            }
        //}
           
        printf("%s",outbuff);
        flag_read_gp2d12 = 0; 
        //usleep(300* 1000);
    }
	close(fd_ir);
}
