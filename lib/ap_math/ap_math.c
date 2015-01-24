#include <math.h>
#include "ap_math.h"


/**
 * a varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is
 * returned.
 * @param  v [description]
 * @return   [description]
 */
float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0;
	}
	if (v >= 1.0f)
	{
		return PI / 2;
	}
	if (v <= -1.0f)
	{
		return -PI / 2;
	}
	return asin(v);
}

/**
 * a varient of sqrt() that checks the input ranges and ensures a
 * valid value as output. If a negative number is given then 0 is
 * returned. The reasoning is that a negative number for sqrt() in our
 * code is usually caused by small numerical rounding errors, so the
 * real input should have been zero
 * @param  v [description]
 * @return   [description]
 */
float safe_sqrt(float v)
{
	float ret = sqrt(v);
	if (isnan(ret))
	{
		return 0;
	}
	return ret;
}

/**
 * constrain a value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
float constrain_float(float amt, float low, float high)
{
	if (isnan(amt))
	{
		return (low + high) * 0.5f;
	}
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/**
 * constrain a int16_t value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
int constrain_int(int amt, int low, int high)
{
	return ((amt) < (low) ? (low) :((amt) > (high) ? (high) : (amt)));
}

/**
 * constrain a int32_t value
 * @param  amt  [description]
 * @param  low  [description]
 * @param  high [description]
 * @return      [description]
 */
long constrain_long(long amt, long low, long high)
{
	return ((amt) < (low) ? (low) :((amt) > (high) ? (high) : (amt)));
}

/**
 * degrees -> radians
 * @param  deg [description]
 * @return     [description]
 */
float radians(float deg)
{
	return deg * DEG_TO_RAD;
}

/**
 * radians -> degrees
 * @param  rad [description]
 * @return     [description]
 */
float degrees(float rad)
{
	return rad * RAD_TO_DEG;
}

/**
 * square
 * @param  v [description]
 * @return   [description]
 */
float sq(float v)
{
	return v * v;
}

/**
 * 2D vector length
 * @param  a [description]
 * @param  b [description]
 * @return   [description]
 */
float pythagorous2(float a, float b)
{
	return sqrt(sq(a) + sq(b));
}

/**
 * 3D vector length
 * @param  a [description]
 * @param  b [description]
 * @param  c [description]
 * @return   [description]
 */
float pythagorous3(float a, float b, float c)
{
	return sqrt(sq(a) + sq(b) + sq(c));
}


