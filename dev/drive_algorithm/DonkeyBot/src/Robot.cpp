/*Changelog: working drive2point, will soon change to point to point to point
 * jun 14, finished checklist, untested
 */

#include "WPILib.h"
#include "motion/path.h"
#include "motion/pathcurve.h"
#include "motion/pathline.h"
#include "motion/pathfollower.h"
#include "SIMPID.h"
#include "AHRS.h"
#define X1 14000
#define X2 10000
#define X3 18000
#define Y 7000

class Robot: public IterativeRobot
{
private:

	Joystick *joystick;

	AHRS *navx;

	Encoder *leftDriveEnc;
	Encoder *rightDriveEnc;

	VictorSP *leftDrive4;
	VictorSP *leftDrive1;
	VictorSP *rightDrive2;
	VictorSP *rightDrive3;

	Path *auto1path1, *auto1path2;


	PathFollower *robot;

	float left;
	float right;
	int autoMode, autoState;

	void RobotInit()
	{
		navx = new AHRS(SPI::Port::kMXP);

		leftDriveEnc = new Encoder(2, 3);
		rightDriveEnc = new Encoder(5, 4);

		leftDrive4 = new VictorSP(4);
		leftDrive1 = new VictorSP(1);
		rightDrive2 = new VictorSP(2);
		rightDrive3 = new VictorSP(3);

		int start[2] = {0, 0};
		int cp1[2] = {X2, 0};
		int cp2[2] = {X3, Y};
		int end1[2] = {X1, 0};
		int end2[2] = {0, Y};


		auto1path1 = new PathLine(start, end1, 10);
		auto1path2 = new PathCurve(end1, cp1, cp2, end2, 10);
		//line3 = new PathLine(vertex, start, 2);
		robot = new PathFollower();
		//line->add(line2);
		//line->add(line3);

		joystick = new Joystick(1);

	}

	void DisabledInit()
	{

	}

	void DisabledPeriodic()
	{
		//printf("Gyro: %f\tLeftEncoder: %d\tRightEncoder: %d", navx->GetYaw(), leftDriveEnc->Get(), rightDriveEnc->Get());
		//printf("X pos: %d\tY pos: %d\n", robot->getXPos(), robot->getYPos());
		robot->updatePos(leftDriveEnc->Get(), rightDriveEnc->Get(), navx->GetYaw());

		if(joystick->GetRawButton(1)){
			robot->reset();
			navx->Reset();
		}
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{
		switch(autoState)
		{
		case 0:
			robot->initPath(auto1path1, PathForward, 0);
			autoState++;
			break;
		case 1:
			if(autoPathDrive())
			{
				autoState++;
				robot->initPath(auto1path2, PathBackward, 180);
			}
			break;
		}
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		printf("Gyro: %f\tLeftEncoder: %d\tRightEncoder: %d", navx->GetYaw(), leftDriveEnc->Get(), rightDriveEnc->Get());
		printf("X pos: %d\tY pos: %d\n", robot->getXPos(), robot->getYPos());
		robot->updatePos(leftDriveEnc->Get(), rightDriveEnc->Get(), navx->GetYaw());

		left = -joystick->GetRawAxis(0) + joystick->GetRawAxis(1);
		right = -joystick->GetRawAxis(0) - joystick->GetRawAxis(1);
		leftDrive4->SetSpeed(left);
		leftDrive1->SetSpeed(left);
		rightDrive2->SetSpeed(right);
		rightDrive3->SetSpeed(right);

		if(joystick->GetRawButton(1)){
			robot->reset();
			navx->Reset();
		}

	}

	void TestPeriodic()
	{

	}

// user functionsssssssssssssssssssssss

	int autoPathDrive()
	{
		if(robot->followPath(leftDriveEnc->Get(), rightDriveEnc->Get(), navx->GetYaw(), left, right) == 0)
		{
			leftDrive4->SetSpeed(left);
			leftDrive1->SetSpeed(left);
			rightDrive2->SetSpeed(right);
			rightDrive3->SetSpeed(right);
		}
		return robot->isDone();
	}
};

START_ROBOT_CLASS(Robot)
