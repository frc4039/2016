#include "WPILib.h"
#include "curve.h"
#include "line.h"
#include "path.h"
#include "pathfollower.h"
#include "AHRS.h"

class Robot: public IterativeRobot
{
private:

	AHRS *nav;

	Encoder *leftDriveEnc;
	Encoder *rightDriveEnc;

	VictorSP *leftDrive4;
	VictorSP *leftDrive1;
	VictorSP *rightDrive2;
	VictorSP *rightDrive3;
	Path *line2;
	PathCurve *something;
	PathFollower *robotFollow;

	void RobotInit()
	{
		int start[2] = {0, 0};
		int end[2] = {5000, 0};

		nav = new AHRS(SPI::Port::kMXP);

		leftDriveEnc = new Encoder(2, 3);
		rightDriveEnc = new Encoder(5, 4);

		leftDrive4 = new VictorSP(4);
		leftDrive1 = new VictorSP(1);
		rightDrive2 = new VictorSP(2);
		rightDrive3 = new VictorSP(3);

		line2 = new PathLine();
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
