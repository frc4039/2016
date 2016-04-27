#include "pathfollower.h"
#include "path.h"
#include <stdio.h>
#include "SimPID.h"



PathFollower::PathFollower(){
	posX = posY = 0;
	lastX = lastY = 0;
	nextPoint = 0;
	leftSpeed = rightSpeed = 0;
	SimPID *PID
}

void PathFollower::initPath(Path *nPath, PathDirection nDirection){

	path = nPath;
	direction = nDirection;
	nextPoint = 0;
	pointPID = new SimPID;
}

bool autoDrive(int distance, int angle)
	{
		int currentDist = (m_rightDriveEncoder->Get() + m_leftDriveEncoder->Get()) / 2;
		int currentAngle = nav->GetYaw();

		drivePID->setDesiredValue(distance);
		turnPID->setDesiredValue(angle);

		float drive = -drivePID->calcPID(currentDist);
		float turn = -turnPID->calcPID(currentAngle);

		m_rightDrive2->SetSpeed(-(limit(drive - turn, 1)));
		m_rightDrive3->SetSpeed(-(limit(drive - turn, 1)));
		m_leftDrive4->SetSpeed(limit(drive + turn, 1));
		m_leftDrive1->SetSpeed(limit(drive + turn, 1));

		return drivePID->isDone() && turnPID->isDone();
	}

void PathFollower::driveToPoint(void){
	//drives to a given point
	//calculates drive output for distance and turn
	//turns towards given point and drives
	//total speed given by PID to endpoint, not target point

	int* nextCoordinate = path->getPoint(nextPoint);
	float desiredAngle = atan2(nextCoordinate[1] - posY, nextCoordinate[0] - posX);
	turnSpeed = turnP*normalize(desiredAngle - angle);


}

float PathFollower::normalize(float normalAngle){
	if(desired Angle)
}

void PathFollower::setSpeed(float nMaxSpeed, float nP){
	maxSpeed = nMaxSpeed;
	distanceP = nP;
}

void PathFollower::followPath(int nPosX, int nPosY, float nAngle, float *nLeftSpeed, float *nRightSpeed){
	posX = nPosX;
	posY = nPosY;
	angle = nAngle;
	driveToPoint();
	//following is acheived by using the drive to point function
	//the robot attempts to drive to a point and when it gets close we move the point
	//to the next point on the path
	//speed is determined by distance from the ENDPOINT not intermediate points
	//use drive to point to do the calculations, this function just controls the targets
	printf("Look at me! I'm Following a path! (%d,%d)\n", posX, posY);
}
