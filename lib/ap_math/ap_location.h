#ifndef AP_LOCATION_H
#define AP_LOCATION_H

struct location
{	
	long alt;		//Altitude in centimeters (meters * 100)
	long lat;		//Lattitude * 10**7
	long lng;		//Longitude * 10**7
};

/**
 * longitude scale 
 * @param  loc [description]
 * @return     [description]
 */
float longitude_scale(const struct location *loc);

/**
 * return distance in meters between two locations
 * @param  loc1 [description]
 * @param  loc2 [description]
 * @return      [description]
 */
float get_distance(const struct location *loc1, const struct location *loc2);

/**
 * get distance in centimeters to between two locations
 * @param  loc1 [description]
 * @param  loc2 [description]
 * @return      [description]
 */
long get_distance_cm(const struct location *loc1, const struct location *loc2);

/**
 * get bearing in centi-degrees between two locations
 * @param  loc1 [description]
 * @param  loc2 [description]
 * @return      [description]
 */
long get_bearing_cd(const struct location *loc1, const struct location *loc2);

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
void location_update(struct location *loc, float bearing, float distance);

/**
 * extrapolate latitude/longitude given distances north and east
 * @param loc       [description]
 * @param ofs_north [description]
 * @param ofs_east  [description]
 */
void location_offset(struct location *loc, float ofs_north, float ofs_east);

/**
 * wrap an angle in centi-degrees to 0..35999
 * @param  error [description]
 * @return       [description]
 */
long wrap_360_cd(long error);

/**
 * wrap an angle in centi-degrees to -18000..18000
 * @param  error [description]
 * @return       [description]
 */
long wrap_180_cd(long error);

/**
 * wrap an angle in centi-degrees to 0..35999
 * @param  angle [description]
 * @return       [description]
 */
float wrap_360_cd_float(float angle);

/**
 * wrap an angle in centi-degrees to -18000..18000
 * @param  angle [description]
 * @return       [description]
 */
float wrap_180_cd_float(float angle);

/**
 * wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 * @param  angle_in_radians [description]
 * @return                  [description]
 */
float wrap_PI(float angle_in_radians);

#endif