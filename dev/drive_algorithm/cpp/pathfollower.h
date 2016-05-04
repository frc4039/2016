#ifndef __PATH_FOLLOWER__
#define __PATH_FOLLOWER__

#include "path.h"

typedef enum PathFollower_directions_enum{
	PathForward = 1,
	PathBackward = -1,
} PathDirection;

class PathFollower
{
private:
	int posX, posY;
	int lastX, lastY;
	int nextPoint;
	float angle;
	PathDirection direction;
	float distanceToEnd;
	float maxSpeed;
	Path *path;
	float leftSpeed, rightSpeed;
	void driveToPoint(void);
	float distanceP;
	float turnP;
	float turnSpeed;
	int lastLeftEnc;
	int lastRightEnc;

public:
	PathFollower();
	void initPath(Path *nPath, PathDirection nDirection);
	void followPath(int nPosX, int nPosY, float nAngle float *nLeftSpeed, float *nRightSpeed);
	void setSpeed(float nMaxSpeed, float nP);
	float normalize(float normalAngle);
	void updatePos(float gyroPos, int leftEnc, int rightEnc, int* x, int* y);

};

#endif
