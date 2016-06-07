#ifndef __PATH_FOLLOWER__
#define __PATH_FOLLOWER__

#include "path.h"
#include "WPILib.h"

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
	float angle;
	PathDirection direction;
	float distanceToEnd;
	float maxSpeed;
	Path *path;
	float leftSpeed, rightSpeed;
	void driveToPoint(void);
	float distanceP;
	float turnP;
	float turnSpeed, driveSpeed;
	float normalize(float normalAngle);

public:
	PathFollower();
	void initPath(Path *nPath, PathDirection nDirection);
	int followPath(int32_t leftEncoder, int32_t rightEncoder, float nAngle, float &nLeftSpeed, float &nRightSpeed);
	void setSpeed(float nMaxSpeed, float nP);
	void updatePos(int leftEncoder, int rightEncoder, float heading);
	void reset(void);
	int getXPos(void);
	int getYPos(void);
	void pickNextPoint(void);
};

#endif
