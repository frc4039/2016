#include "WPILib.h"
#include <USBCamera.h>
#include <math.h>
#include "USBVision.h"

#define RES_X 640
#define RES_Y 480

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	Victor *m_frontLeftDrive; //0
	Victor *m_frontRightDrive; //1
	Victor *m_rearLeftDrive; //2
	Victor *m_rearRightDrive; //3

	Joystick *m_Joystick;
	USBVision *vision;

	void RobotInit() override
	{
		printf("starting robot!\n");
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);

		m_frontLeftDrive = new Victor(0);
		m_frontRightDrive = new Victor(1);
		m_rearLeftDrive = new Victor(2);
		m_rearRightDrive = new Victor(3);

		m_Joystick = new Joystick(0);

		vision = new USBVision((Int64)10, (Int64)220, (Int64)10, (Int64)80);
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{
		autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom)
		{
			//Custom Auto goes here
		}
		else
		{
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom)
		{
			//Custom Auto goes here
		}
		else
		{
			//Default Auto goes here
		}
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		vision->getProcessedImage();
		lw->Run();
	}

	void teleDrive()
	{
		float x = m_Joystick->GetX();
		float y = m_Joystick->GetY();

		printf("Joystick x=%f, y=%f\n", x,y);
		//float leftSpeed = x + y;
		//float rightSpeed = -x + y;
	}



	void DisabledInit()
	{
	}

	void DisabledPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
