#include <stdio.h> 
#include "ap_math.h"

int main()
{
	printf("asin of 1 is %f \n", safe_asin(1));
	printf("sqrt of 4 is %f \n", safe_sqrt(4));
	printf("180 degree is %f rad\n", radians(180));
	printf("PI rad is %f degree\n", degrees(PI));
	printf("square of 3 is %f \n", sq(3));
	float m = 100;
	float low = 0;
	float high = 180;
	printf("constrain_float(%f,%f, %f) is %f \n", m, low, high, constrain_float(m, low, high));

	int n = 190;
	int low_int = 0;
	int high_int = 180;
	printf("constrain_int(%d,%d, %d) is %d \n", n, low_int, high_int, constrain_int(n, low_int, high_int));

	long x = 1;
	long low_log = 0;
	long high_log = 180;
	printf("constrain_long(%d,%d, %d) is %d \n", x, low_log, high_log, constrain_long(x, low_log, high_log));
	printf("length of vector(3, 4) is %f \n", pythagorous2(3, 4));
	printf("length of vector(1, 1, 1) is %f \n", pythagorous3(1, 1, 1));
}
