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
	nextPoint = 0;
}

void PathFollower::driveToPoint(void){
	//drives to a given point
	//calculates drive output for distance and turn
	//turns towards given point and drives
	//total speed given by PID to endpoint, not target point

	int* nextCoordinate = path->getPoint(nextPoint);
	float desiredAngle = atan2((float)(nextCoordinate[1] - posY), (float)(nextCoordinate[0] - posX));
	turnSpeed = turnP*normalize(desiredAngle - angle);


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


int PathFollower::followPath(int nPosX, int nPosY, float nAngle, float *nLeftSpeed, float *nRightSpeed){
	posX = nPosX;
	posY = nPosY;
	angle = nAngle;
	driveToPoint();

	float driveSpeed = distanceP*sqrt((float)((SQ(path->getEndPoint()[0]-posX) + SQ(path->getEndPoint()[1]-posY))));


	//following is acheived by using the drive to point function
	//the robot attempts to drive to a point and when it gets close we move the point
	//to the next point on the path
	//speed is determined by distance from the ENDPOINT not intermediate points
	//use drive to point to do the calculations, this function just controls the targets
	printf("Look at me! I'm Following a path! (%d,%d)\n", posX, posY);
	return 0;
}

void PathFollower::updatePos(void){

}
