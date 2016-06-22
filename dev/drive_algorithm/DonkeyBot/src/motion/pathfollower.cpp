#include "pathfollower.h"
#include "path.h"

#include <stdio.h>
#include <math.h>

#include "SimPID.h"
#include "shiftlib.h"

using namespace shiftlib;


PathFollower::PathFollower(){
	posX = posY = 0;
	lastX = lastY = 0;
	nextPoint = 0;
	leftSpeed = rightSpeed = 0;

}

void PathFollower::initPath(Path *nPath, PathDirection nDirection, float nFinalAngleDegrees){

	path = nPath;
	direction = nDirection;
	nextPoint = 1;
	finalAngle = deg2rad(nFinalAngleDegrees);

	done = false;
	driveDone = false;

	//this stuff to constructor
	maxSpeed = 0.5;
	distanceP = 0.0001;
	turnP = 0.825;

	//use these 4 values to tune the drive
	maxTurnError = PI/3; //the  error value in radians where the drive stops moving
	distanceError = 500;

	turnPID = new SimPID(0.85, 0, 0.02, 0.087266);
	turnPID->setContinuousAngle(true);

	drivePID = new SimPID(0.0005,0, 0.0002, 0);
	drivePID->setMaxOutput(0.8);
}

void PathFollower::driveToPoint(void){
	//drives to a given point
	//calculates drive output for distance and turn
	//turns towards given point and drives
	//total speed given by PID to endpoint, not target point

	int* nextCoordinate = path->getPoint(nextPoint);
	float desiredAngle = atan2((float)(nextCoordinate[1] - posY), (float)(nextCoordinate[0] - posX));

	if (direction == PathBackward)
	{
		desiredAngle += PI;
	}

	turnPID->setDesiredValue(normalizeRad(desiredAngle));
	turnSpeed = turnPID->calcPID(angle);

	//printf("driving to point (%d,%d,%d)\n", nextCoordinate[0], nextCoordinate[1], nextPoint);
	//printf("desiredangle: %f", desiredAngle*180/PI);
}


void PathFollower::setSpeed(float nMaxSpeed, float nP){
	maxSpeed = nMaxSpeed;
	distanceP = nP;
}


void PathFollower::pickNextPoint(void){
	distanceToPoint = sqrt((float)((SQ(path->getPoint(nextPoint)[0]-posX) + SQ(path->getPoint(nextPoint)[1]-posY))));

	if(distanceToPoint < distanceError && nextPoint == path->size - 1)
			driveDone = true;
	if(distanceToPoint < distanceError && nextPoint + 1 != path->size)
	{
		nextPoint = nextPoint + 1;

	}
}

int PathFollower::followPath(int32_t leftEncoder, int32_t rightEncoder, float nAngle, float &nLeftSpeed, float &nRightSpeed){
	//following is acheived by using the drive to point function
	//the robot attempts to drive to a point and when it gets close we move the point
	//to the next point on the path
	//speed is determined by distance from the ENDPOINT not intermediate points
	//use drive to point to do the calculations, this function just controls the targets

	angle = deg2rad(nAngle);

	driveError = sqrt((float)((SQ(path->getEndPoint()[0]-posX) + SQ(path->getEndPoint()[1]-posY))));

	//get new position
	updatePos(leftEncoder, rightEncoder, nAngle);

	pickNextPoint();

	//get drive speed and turn speed
	if(path->getPathDistance(nextPoint) > driveError)
	{
		driveError = path->getPathDistance(nextPoint);
	}

	drivePID->setDesiredValue(0);


	driveSpeed = direction*drivePID->calcPID(driveError) * limit(1 - fabs(turnPID->getError())/maxTurnError, 1.f);

	driveToPoint();


	//set left drive and right drive variables
	//this automatically returns them

	if(driveDone == false)
	{
		nLeftSpeed = -turnSpeed + driveSpeed;
		nRightSpeed = -turnSpeed - driveSpeed;
	}
	else if(done == false){
		turnSpeed = driveToAngle();

		nLeftSpeed = -turnSpeed;
		nRightSpeed	= -turnSpeed;

		if(turnPID->isDone())
			done = true;
	}
	else
	{
		nLeftSpeed = 0;
		nRightSpeed = 0;
	}

	printf("angle: %f\tPosX: %d\tPosY: %d\n", nAngle, posX, posY);
	printf("turn: %f\tdrive: %f\n", turnSpeed, driveSpeed);

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
	nextPoint = 1;
	driveDone = false;
	done = false;
}

int PathFollower::getXPos(void){
	return posX;
}

int PathFollower::getYPos(void){
	return posY;
}

float PathFollower::driveToAngle(void){
	turnPID->setDesiredValue(finalAngle);
	return turnPID->calcPID(angle);
}

bool PathFollower::isDone()
{
	return done;
}

