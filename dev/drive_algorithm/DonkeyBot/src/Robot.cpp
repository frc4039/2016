#include "WPILib.h"
#include "motion/path.h"
#include "motion/pathcurve.h"
#include "motion/pathline.h"
#include "motion/pathfollower.h"
//#include "AHRS.h"

class Robot: public IterativeRobot
{
private:

	//AHRS *navx;

	Encoder *leftDriveEnc;
	Encoder *rightDriveEnc;

	VictorSP *leftDrive4;
	VictorSP *leftDrive1;
	VictorSP *rightDrive2;
	VictorSP *rightDrive3;

	Path *line;

	void RobotInit()
	{
		//navx = new AHRS(SPI::Port::kMXP);

		leftDriveEnc = new Encoder(2, 3);
		rightDriveEnc = new Encoder(5, 4);

		leftDrive4 = new VictorSP(4);
		leftDrive1 = new VictorSP(1);
		rightDrive2 = new VictorSP(2);
		rightDrive3 = new VictorSP(3);

		int start[2] = {0, 0};
		int end[2] = {5000, 0};
		line = new PathLine(start, end, 2);
	}

	void DisabledInit()
	{

	}

	void DisabeledPeriodic()
	{

	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)
