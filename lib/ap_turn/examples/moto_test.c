#include "ap_turn.h"
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
int main()
{
	int init_speed=300;
	moto_init(init_speed);
	int speed=500;
	int increment=-200;
	while(1)
	{
	moto_turn(speed,increment);
	signal(SIGINT,moto_stop);
	}
	return 0;
	
}