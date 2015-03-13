#include <stdio.h>
#include <linux/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <math.h>
int fd_pwm,fd_setpin;
void moto_init(int init_speed)
{

	fd_pwm=open("/dev/iopwm",O_RDWR);
	fd_setpin=open("/dev/SetPin",O_RDWR);
	if(fd_pwm<0)
                {
                        perror("open pwm error");
                }
     if(fd_setpin<0)
                {
                        perror("open SetPin error");
                }
                 
	if(init_speed>290&&init_speed<650)
	{
	ioctl(fd_setpin,0,1);
	ioctl(fd_pwm,1,init_speed);
	ioctl(fd_pwm,0,init_speed);
	}
	else
	{
		  perror("speed error");
	}
}
void moto_turn(int speed,int increment)
{
	if(speed>100&&speed<900&&speed>=abs(increment)&&(speed+increment)<900)  //900速度就驱动不起电机了
	{
		
			ioctl(fd_pwm,1,increment+speed);
			ioctl(fd_pwm,0,speed-increment);

		if(increment>0&&increment<500)
		{
			
			fprintf(stdout, "The car is turning left\n");
		} 
		 if(increment<0&&increment>-500)
			{
			
			fprintf(stdout, "The car is turning right\n");
			}
			if(increment==0)
			{
			
			fprintf(stdout, "The car is walking straight\n");
			}
		
	}
	else
		fprintf(stdout, "The speed set error\n");
	
	
}
void moto_stop()
{
	ioctl(fd_setpin,4,1);
	close(fd_setpin);
	close(fd_pwm);
	//close(fd_d);
	_exit(0);
}