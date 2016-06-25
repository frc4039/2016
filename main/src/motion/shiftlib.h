#ifndef __MAKESHIFT_LIB__
#define __MAKESHIFT_LIB__

#include "path.h"
#include "pathline.h"
#include "pathcurve.h"
#include "pathfollower.h"
#include "SimPID.h"

#define SQ(X) ((X)*(X))
#define PI 3.141592653589793f

namespace shiftlib {
	
	//converts degrees to radians
	inline float deg2rad(float deg){
		return deg / 180.f * PI;
	}
	
	//converts radians to degrees
	inline float rad2deg(float rad){
		return rad / PI * 180.f;
	}
	
	//raises x to an exponent n, always keeping negative sign
	inline float expo(float x, int n)
	{
		int sign = n % 2;
		float y = 1;
		for (int i = 1; i <= n; i++)
		{
			y *= x;
		}
		if(sign == 0 && x < 0.f)
			return -y;
		return y;
	}
	
	//multiplies x by a scaling factor scale
	inline float scale(float x, float scale){
		return x * scale;
	}

	//limits x: -lim <= x <= lim
	inline float limit(float x, float lim)
	{
		if (x > lim)
			return lim;
		else if (x < -lim)
			return -lim;
		return x;
	}
	
	//normalizes radians to be >= -PI or <= PI
	inline float normalizeRad(float normalAngle){
	while(normalAngle > PI)
		normalAngle -= 2.f*PI;
	
	while(normalAngle < -PI)
		normalAngle += 2.f*PI;
	
	return normalAngle;
	}
	
	//normalizes radians to be >= -PI or <= PI
	inline float normalizeDeg(float normalAngle){
	while(normalAngle > 180.f)
		normalAngle -= 360.f;
	
	while(normalAngle < -180.f)
		normalAngle += 360.f;
	
	return normalAngle;
	}
}

#endif
