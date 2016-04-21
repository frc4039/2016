#include "pathfollower.h"
#include "path.h"
#include <stdio.h>

PathFollower::PathFollower(){
	posX = posY = 0;
	lastX = lastY = 0;
	nextPoint = 0;
	leftSpeed = rightSpeed = 0;
}

void PathFollower::initPath(Path *nPath, PathDirection nDirection){
	path = nPath;
	direction = nDirection;
	nextPoint = 0;
}

void PathFollower::driveToPoint(void){
	//drives to a given point
	//calculates drive output for distance and turn
	//turns towards given point and drives
	//total speed given by PID to endpoint, not target point
}

void PathFollower::setSpeed(float nMaxSpeed, float nP){
	maxSpeed = nMaxSpeed;
	distanceP = nP;
}

void PathFollower::followPath(int nPosX, int nPosY, float *nLeftSpeed, float *nRightSpeed){
	posX = nPosX;
	posY = nPosY;
	//following is acheived by using the drive to point function
	//the robot attempts to drive to a point and when it gets close we move the point
	//to the next point on the path
	//speed is determined by distance from the ENDPOINT not intermediate points
	//use drive to point to do the calculations, this function just controls the targets
	printf("Look at me! I'm Following a path! (%d,%d)\n", posX, posY);
}
