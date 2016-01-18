#include "WPILib.h"
#include <USBCamera.h>

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

	Image *image;
	USBCamera *Camera;
	RGBValue *colours;

	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;


	void RobotInit() override
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);

		m_frontLeftDrive = new Victor(0);
		m_frontRightDrive = new Victor(1);
		m_rearLeftDrive = new Victor(2);
		m_rearRightDrive = new Victor(3);

		m_Joystick = new Joystick(0);

		Camera = new USBCamera("cam0", true);

		//intermediate vision

		// create an image
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);

		if(imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}

		imaqError = IMAQdxConfigureGrab(session);

		if(imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}

		/*Camera->SetFPS(1);
		Camera->SetExposureManual(1);
		Camera->SetWhiteBalanceManual(1);
		Camera->SetBrightness(1);
		*/

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
		Camera->SetExposureManual(1);
		Camera->SetWhiteBalanceManual(1);
		Camera->SetBrightness(1);
		Camera->SetFPS(1);
		CaptureVideo();
		lw->Run();
	}

	bool teleDrive()
	{
		float x = m_Joystick->GetX();
		float y = m_Joystick->GetY();

		printf("Joystick x=%f, y=%f\n", x,y);
		//float leftSpeed = x + y;
		//float rightSpeed = -x + y;
	}

	void CaptureVideo()
	{
		// acquire images
		IMAQdxStartAcquisition(session);

	    // grab an image, draw the circle, and provide it for the camera server which will
	    // in turn send it to the dashboard.
		while(IsOperatorControl() && IsEnabled())
		{
			IMAQdxGrab(session, frame, true, NULL);

			if(imaqError != IMAQdxErrorSuccess)
			{
				DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
			}
			else
			{
				imaqDrawShapeOnImage(frame, frame, { 10, 10, 200, 200 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
				CameraServer::GetInstance()->SetImage(frame);
			}

			Wait(0.005);				// wait for a motor update time
		}

		// stop image acquisition
		IMAQdxStopAcquisition(session);
		}

	void TestPeriodic()
	{
		lw->Run();
	}

	void DisabledInit()
	{
	}

	void DisabledPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
