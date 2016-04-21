#ifndef __PATH_FOLLOWER__
#define __PATH_FOLLOWER__

#include "path.h"

class PathFollower
{
private:
	int posX, posY;
	int lastX, lastY;
	int nextPoint;
	float distanceToEnd;
	float maxSpeed;
	Path *path;
	float leftSpeed, rightSpeed;
	void driveToPoint(void);
	float distanceP;

public:
	PathFollower();
	void initPath(Path *nPath);
	void followPath(int nPosX, int nPosY, float *nLeftSpeed, float *nRightSpeed);
	void setSpeed(float nMaxSpeed, float nP);

};

#endif
