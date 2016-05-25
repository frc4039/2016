#include "pathfollower.h"
#include "path.h"
#include <stdio.h>

#include <math.h>



PathFollower::PathFollower(){
	posX = posY = 0;
	lastX = lastY = 0;
	nextPoint = 0;
	leftSpeed = rightSpeed = 0;
}

void PathFollower::initPath(Path *nPath, PathDirection nDirection){

	path = nPath;
	direction = nDirection;
	nextPoint = 1;
}

void PathFollower::driveToPoint(void){
	//drives to a given point
	//calculates drive output for distance and turn
	//turns towards given point and drives
	//total speed given by PID to endpoint, not target point

	int* nextCoordinate = path->getPoint(nextPoint);
	float desiredAngle = atan2((float)(nextCoordinate[1] - posY), (float)(nextCoordinate[0] - posX));
	turnSpeed = turnP*normalize(desiredAngle - angle);

	printf("driving to point (%d,%d)\n", nextCoordinate[0], nextCoordinate[1]);
}

float PathFollower::normalize(float normalAngle){

	if(normalAngle > 180)

		normalAngle -= 360;

	else if(normalAngle < -180)

		normalAngle += 360;

	return normalAngle;
}

void PathFollower::setSpeed(float nMaxSpeed, float nP){
	maxSpeed = nMaxSpeed;
	distanceP = nP;
}

#define SQ(X) ((X)*(X))


int PathFollower::followPath(int leftEncoder, int rightEncoder, float nAngle, float &nLeftSpeed, float &nRightSpeed){
	//following is acheived by using the drive to point function
	//the robot attempts to drive to a point and when it gets close we move the point
	//to the next point on the path
	//speed is determined by distance from the ENDPOINT not intermediate points
	//use drive to point to do the calculations, this function just controls the targets

	//angle = nAngle;

	//get new position
	updatePos(leftEncoder, rightEncoder, nAngle);

	//get drive speed and turn speed
	driveSpeed = distanceP*sqrt((float)((SQ(path->getEndPoint()[0]-posX) + SQ(path->getEndPoint()[1]-posY))));
	driveToPoint();

	//set left drive and right drive variables
	//this automatically returns them
	nLeftSpeed = -turnSpeed + driveSpeed;
	nRightSpeed = -turnSpeed - driveSpeed;

	printf("driving to path. turn: %f\tdrive: %f)\n", turnSpeed, driveSpeed);

	//return 1 if follow is complete, else 0
	return 0;
}

void PathFollower::updatePos(int leftEncoder, int rightEncoder, float direction){
	//this function will update the robots position, is called periodically by followPath()
	int d = ((leftEncoder - lastLeftEncoder) + (rightEncoder - lastRightEncoder) ) / 2;

	posX += d * cos(direction);
	posY += d * sin(direction);

	lastLeftEncoder = leftEncoder;
	lastRightEncoder = rightEncoder;
}
