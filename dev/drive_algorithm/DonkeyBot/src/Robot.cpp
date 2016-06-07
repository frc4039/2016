//Changelog: working drive2point, will soon change to point to point to point

#include "WPILib.h"
#include "motion/path.h"
#include "motion/pathcurve.h"
#include "motion/pathline.h"
#include "motion/pathfollower.h"
#include "AHRS.h"

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

	Path *line;

	PathFollower *robot;

	float left;
	float right;

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
		int end[2] = {-4000, -8000};
		line = new PathLine(start, end, 2);

		robot = new PathFollower();

		joystick = new Joystick(1);

	}

	void DisabledInit()
	{

	}

	void DisabledPeriodic()
	{
		printf("Gyro: %f\tLeftEncoder: %d\tRightEncoder: %d", navx->GetYaw(), leftDriveEnc->Get(), rightDriveEnc->Get());
		printf("X pos: %d\tY pos: %d\n", robot->getXPos(), robot->getYPos());
		robot->updatePos(leftDriveEnc->Get(), rightDriveEnc->Get(), navx->GetYaw());

		if(joystick->GetRawButton(1)){
			robot->reset();
			navx->Reset();
		}
	}

	void AutonomousInit()
	{
		robot->initPath(line, PathForward);
	}

	void AutonomousPeriodic()
	{
		if(robot->followPath(leftDriveEnc->Get(), rightDriveEnc->Get(), navx->GetYaw(), left, right) == 0)
		{
			leftDrive4->SetSpeed(left);
			leftDrive1->SetSpeed(left);
			rightDrive2->SetSpeed(right);
			rightDrive3->SetSpeed(right);
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
};

START_ROBOT_CLASS(Robot)
