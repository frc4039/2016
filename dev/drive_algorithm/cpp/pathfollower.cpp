#include "pathfollower.h"
#include "path.h"
#include <stdio.h>

PathFollower::PathFollower(){
	posX = posY = 0;
	lastX = lastY = 0;
	nextPoint = 0;
	leftSpeed = rightSpeed = 0;
}

void PathFollower::initPath(Path *nPath){
	path = nPath;
	nextPoint = 0;
}

void PathFollower::driveToPoint(void){

}

void PathFollower::setSpeed(float nMaxSpeed, float nP){
	maxSpeed = nMaxSpeed;
	distanceP = nP;
}

void PathFollower::followPath(int nPosX, int nPosY, float *nLeftSpeed, float *nRightSpeed){
	posX = nPosX;
	posY = nPosY;
	printf("Look at me! I'm Following a path! (%d,%d)\n", posX, posY);
}
