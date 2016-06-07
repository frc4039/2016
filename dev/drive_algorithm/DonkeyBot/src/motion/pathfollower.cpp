#include "pathfollower.h"
#include "path.h"
#include "WPILib.h"
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

	maxSpeed = 0.5;
	distanceP = 0.0001;
	turnP = 0.825;
}

void PathFollower::driveToPoint(void){
	//drives to a given point
	//calculates drive output for distance and turn
	//turns towards given point and drives
	//total speed given by PID to endpoint, not target point

	int* nextCoordinate = path->getPoint(nextPoint);
	float desiredAngle = atan2((float)(nextCoordinate[1] - posY), (float)(nextCoordinate[0] - posX));
	
	if (direction == PathForwards)
		turnSpeed = turnP * normalize(desiredAngle - angle);
	else
		turnSpeed = turnP * normalize(desiredAngle - angle + PI);
	printf("driving to point (%d,%d)\n", nextCoordinate[0], nextCoordinate[1]);
}
#define PI 3.141592653589793

float PathFollower::normalize(float normalAngle){

	if(normalAngle > PI)

		normalAngle -= 2*PI;

	else if(normalAngle < -PI)

		normalAngle += 2*PI;

	return normalAngle;
}

void PathFollower::setSpeed(float nMaxSpeed, float nP){
	maxSpeed = nMaxSpeed;
	distanceP = nP;
}

#define SQ(X) ((X)*(X))
float deg2rad(float deg){
	return deg / 180 * PI;
}

void pickNextPoint(void){

	nextPoint = ?????;
}

int PathFollower::followPath(int32_t leftEncoder, int32_t rightEncoder, float nAngle, float &nLeftSpeed, float &nRightSpeed){
	//following is acheived by using the drive to point function
	//the robot attempts to drive to a point and when it gets close we move the point
	//to the next point on the path
	//speed is determined by distance from the ENDPOINT not intermediate points
	//use drive to point to do the calculations, this function just controls the targets

	angle = deg2rad(nAngle);

	//get new position
	updatePos(leftEncoder, rightEncoder, nAngle);

	pickNextPoint();

	//get drive speed and turn speed
	driveSpeed = -direction*distanceP*sqrt((float)((SQ(path->getEndPoint()[0]-posX) + SQ(path->getEndPoint()[1]-posY))));
	driveToPoint();

	if (driveSpeed > maxSpeed){
		driveSpeed = maxSpeed;
	}
	else if (driveSpeed < -maxSpeed){
		driveSpeed = -maxSpeed;
	}

	//set left drive and right drive variables
	//this automatically returns them
	nLeftSpeed = -turnSpeed + driveSpeed;
	nRightSpeed = -turnSpeed - driveSpeed;

	printf("angle: %f\tPosX: %d\tPosY: %d\n", nAngle, posX, posY);
	printf("driving to path. turn: %f\tdrive: %f)\n", turnSpeed, driveSpeed);

	//return 1 if follow is complete, else 0
	return 0;
}



void PathFollower::updatePos(int leftEncoder, int rightEncoder, float heading){
	//this function will update the robots position, is called periodically by followPath()
	int d = ((leftEncoder - lastLeftEncoder) + (rightEncoder - lastRightEncoder) ) / 2;

	posX += d * cos(deg2rad(heading));
	posY += d * sin(deg2rad(heading));

	lastLeftEncoder = leftEncoder;
	lastRightEncoder = rightEncoder;
}

void PathFollower::reset(void)
{
	posX = 0;
	posY = 0;
}

int PathFollower::getXPos(void){
	return posX;
}

int PathFollower::getYPos(void){
	return posY;
}

