/*Changelog: working drive2point, will soon change to point to point to point
 * jun 14, finished checklist, untested
 */

#include "WPILib.h"
#include "motion/path.h"
#include "motion/pathcurve.h"
#include "motion/pathline.h"
#include "motion/pathfollower.h"
//#include "SimPID.h"
#include "AHRS.h"
#define X1 -14000
#define X2 -10000
#define X3 -18000
#define Y1 7000
#define Y2

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

	Path *auto1path1, *auto1path2, *auto2path1, *auto2path2, *auto2path3, *auto2path4, *auto2path5, *auto2path6;


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

		/* figure 8
		int start[2] = {0, 0};
		int cp1[2] = {10000, 0};
		int cp2[2] = {-10000, 14000};
		int cp3[2] = {10000, 14000};
		int cp4[2] = {-10000, 0};
		int end1[2] = {0, 14000};
		int end2[2] = {0, 7000};
		*/

/*		int start[2] = {0, 0};
		int end1[2] = {X1, 0};

		int cp1[2] = {X2, 0};
		int cp2[2] = {X3, Y};
		int end2[2] = {0, Y};*/

		int start[2] = {0, 0};
		int end1[2] = {14000, 0};

		int cp1[2] = {20000, -4000};
		int cp2[2] = {24000, 0};
		int end2[2] = {20000, 4000};

		//auto1path1 = new PathLine(start, end1, 10);
		//auto1path2 = new PathCurve(end1, cp1, cp2, end2, 10);
		//line3 = new PathLine(vertex, start, 2);
		//auto1path1->add(auto1path2);
		auto2path1 = new PathLine(start, end1, 10);
		auto2path2 = new PathCurve(end1, cp2, cp1, end2, 10);
		auto2path1->add(auto2path2);
		auto2path3 = new PathCurve(end2, cp1, cp2, end1, 10);
		auto2path4 = new PathLine(end1, start, 10);
		auto2path3->add(auto2path4);
		auto2path5 = new PathLine(start, end1, 10);
		auto2path6 = new PathCurve(end1, cp1, cp2, end2, 10);

		robot = new PathFollower();

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
			autoState = 0;
		}
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{
		//printf("AutoState: %d", autoState);
		switch(autoState)
		{
		case 0:
			robot->initPath(auto2path1, PathForward, 0);
			autoState++;
			break;
		case 1:
			if(autoPathDrive())
			{
				autoState++;
				robot->initPath(auto2path3, PathBackward, 0);
			}
			break;
		case 2:
			if(autoPathDrive())
			{
				autoState++;
				robot->initPath(auto2path1, PathForward, 0);
			}
			break;
		case 3:
			autoPathDrive();
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
