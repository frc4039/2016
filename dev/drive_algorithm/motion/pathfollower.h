#ifndef __PATH_FOLLOWER__
#define __PATH_FOLLOWER__

#include "path.h"

#include "SimPID.h"

#include <stdint.h>

typedef enum PathFollower_directions_enum{
	PathForward = 1,
	PathBackward = -1,
} PathDirection;

class PathFollower
{
private:
	int posX, posY;
	int lastX, lastY;
	int lastLeftEncoder, lastRightEncoder;
	int nextPoint;
	bool driveDone;
	bool done;
	float angle;
	float finalAngle;
	PathDirection direction;
	float distanceToEnd;
	float distanceToPoint;
	float distanceError;
	float turnError;
	float driveError;
	float maxSpeed;
	Path *path;
	float leftSpeed, rightSpeed;
	void driveToPoint(void);
	float driveToAngle(void);
	float distanceP;
	float turnP;
	float maxTurnError;
	float turnSpeed, driveSpeed;
	float normalize(float normalAngle);


public:
	PathFollower();
	void initPath(Path *nPath, PathDirection nDirection, float nFinalAngleDegrees);
	int followPath(int32_t leftEncoder, int32_t rightEncoder, float nAngle, float &nLeftSpeed, float &nRightSpeed);
	void setSpeed(float nMaxSpeed, float nP);
	void updatePos(int leftEncoder, int rightEncoder, float heading);
	void reset(void);
	int getXPos(void);
	int getYPos(void);
	void pickNextPoint(void);
	bool isDone(void);
	SimPID *turnPID;
	SimPID *drivePID;
};

#endif
