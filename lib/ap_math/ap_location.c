#include <math.h>
#include <stdlib.h>
#include "ap_math.h"
#include "ap_location.h"
// radius of earth in meters 
#define RADIUS_OF_EARTH 6378100

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

/**
 * longitude scale 
 * @param  loc [description]
 * @return     [description]
 */
float longitude_scale(const struct location *loc)
{
	static long last_lat;
	static float scale = 1.0;
	if (abs(last_lat - loc->lat) < 100000)
	{
		return scale;
	}
	scale = cos(loc->lat * 1.0e-7f * DEG_TO_RAD);
	scale = constrain_float(scale, 0.01f, 1.0f);
	last_lat = loc->lat;
	return scale;
}

/**
 * return distance in meters between two locations
 * @param  loc1 [description]
 * @param  loc2 [description]
 * @return      [description]
 */
float get_distance(const struct location *loc1, const struct location *loc2)
{
	float dlat = (float)(loc2->lat - loc1->lat);
	float dlong = ((float)(loc2->lng - loc1->lng)) * longitude_scale(loc2);
	return pythagorous2(dlat, dlong) * LOCATION_SCALING_FACTOR;
}

/**
 * get distance in centimeters to between two locations
 * @param  loc1 [description]
 * @param  loc2 [description]
 * @return      [description]
 */
long get_distance_cm(const struct location *loc1, const struct location *loc2)
{
	return (long)(get_distance(loc1, loc2) * 100);
}

/**
 * get bearing in centi-degrees between two locations
 * @param  loc1 [description]
 * @param  loc2 [description]
 * @return      [description]
 */
long get_bearing_cd(const struct location *loc1, const struct location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) / longitude_scale(loc2);
	long bearing = 9000 + (long)(atan2(-off_y, off_x) * 5729.57795f);
	if (bearing < 0)
	{
		bearing += 36000;
	}
	return bearing;
}


//int location_passed_point(const struct location location, const struct location point1)

/**
 * extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of 
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 * @param loc      [description]
 * @param bearing  [description]
 * @param distance [description]
 */
void location_update(struct location *loc, float bearing, float distance)
{
	float ofs_north = cos(radians(bearing)) * distance;
	float ofs_east = sin(radians(bearing)) * distance;
	location_offset(loc, ofs_north, ofs_east);
}

/**
 * extrapolate latitude/longitude given distances north and east
 * @param loc       [description]
 * @param ofs_north [description]
 * @param ofs_east  [description]
 */
void location_offset(struct location *loc, float ofs_north, float ofs_east)
{
	if (ofs_north != 0 || ofs_east != 0)
	{
		long dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
		long dlng = (ofs_east) * LOCATION_SCALING_FACTOR_INV / longitude_scale(loc);
		loc->lat += dlat;
		loc->lng += dlng;
	}
}

/**
 * wrap an angle in centi-degrees to 0..35999
 * @param  error [description]
 * @return       [description]
 */
long wrap_360_cd(long error)
{
	if (error > 360000 || error < -360000)
	{
		error = error % 36000;
	}
	while (error >= 36000)
	{
		error -= 36000;
	}
	while (error <0)
	{
		error += 36000;
	}
	return error;
}

/**
 * wrap an angle in centi-degrees to -18000..18000
 * @param  error [description]
 * @return       [description]
 */
long wrap_180_cd(long error)
{
	if (error > 360000 || error < -360000)
	{
		error = error % 36000;
	}
	while (error >= 18000)
	{
		error -= 36000;
	}
	while (error < -18000)
	{
		error += 36000;
	}
	return error;
}

/**
 * wrap an angle in centi-degrees to 0..35999
 * @param  angle [description]
 * @return       [description]
 */
float wrap_360_cd_float(float angle)
{
	if (angle > 360000 || angle < -360000)
	{
		angle = fmod(angle, 36000.0f);
	}
	while (angle >= 36000.0f)
	{
		angle -= 36000.0f;
	}
	while (angle <0.0f)
	{
		angle += 36000.0f;
	}
	return angle;
}

/**
 * wrap an angle in centi-degrees to -18000..18000
 * @param  angle [description]
 * @return       [description]
 */
float wrap_180_cd_float(float angle)
{
    if (angle > 54000.0f || angle < -54000.0f) 
    {
        // for large numbers use modulus
        angle = fmod(angle,36000.0f);
    }
    if (angle > 18000.0f) 
    { 
    		angle -= 36000.0f; 
    }
    if (angle < -18000.0f) 
    { 
    	angle += 36000.0f; 
    }
    return angle;
}

/**
 * wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 * @param  angle_in_radians [description]
 * @return                  [description]
 */
float wrap_PI(float angle_in_radians)
{
    if (angle_in_radians > 10*PI || angle_in_radians < -10*PI) 
    {
        // for very large numbers use modulus
        angle_in_radians = fmod(angle_in_radians, 2*PI);
    }
    while (angle_in_radians > PI)
    {
    	angle_in_radians -= 2*PI;    	
    } 

    while (angle_in_radians < -PI) 
    {
    	angle_in_radians += 2*PI;
    }
    		
    return angle_in_radians;
}