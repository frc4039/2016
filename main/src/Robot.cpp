#include "WPILib.h"
#include <USBCamera.h>
#include <math.h>

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

	//=======================Vision Variables======================
	USBCamera *Camera;
	IMAQdxSession session;

	Image *frame;
	Image *processed;
	ImageInfo raw_info;
	ImageInfo proc_info;
	IMAQdxError imaqError;
	char *raw_pixel;
	char *proc_pixel;


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

		VisionInit();


	}

	void getImageInfo(void){
		imaqGetImageInfo(frame, &raw_info);
		imaqGetImageInfo(processed, &proc_info);
		raw_pixel = (char*)(raw_info.imageStart);
		proc_pixel = (char*)(proc_info.imageStart);
	}

#define RES_X 640
#define RES_Y 480

	void VisionInit(void){

		Camera = new USBCamera("cam0", true);

		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		processed = imaqCreateImage(ImageType::IMAQ_IMAGE_U8,0);

		imaqResample(frame, frame, RES_X, RES_Y, InterpolationMethod::IMAQ_ZERO_ORDER, {0,0,RES_X,RES_Y});
		imaqResample(processed, processed, RES_X, RES_Y, InterpolationMethod::IMAQ_ZERO_ORDER, {0,0,RES_X,RES_Y});

		getImageInfo();
		printf("Picture resolution is %d, %d\n", raw_info.xRes, raw_info.yRes);
		printf("pixels per line: %d\n", raw_info.pixelsPerLine);


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
		Camera->SetExposureManual(100);
		Camera->SetWhiteBalanceManual(100);
		Camera->SetBrightness(100);
		//Camera->SetFPS(1);
		TakePicture();
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

	void TakePicture()
	{
		// acquire images
		IMAQdxStartAcquisition(session);

	    // grab an image, draw the circle, and provide it for the camera server which will
	    // in turn send it to the dashboard.
		while(IsOperatorControl() && IsEnabled())
		{
			IMAQdxGrab(session, frame, true, NULL);
			getImageInfo();

			if(imaqError != IMAQdxErrorSuccess)
			{
				DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
			}
			else
			{
				//imaqDrawShapeOnImage(frame, frame, { 10, 10, 200, 200 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
				/*
				Threshold t;
				HSLImage target;
				BinaryImage b = target.t;
				*/


				HSLFilter();


				CameraServer::GetInstance()->SetImage(frame);

			}

			Wait(0.005);				// wait for a motor update time
		}

		// stop image acquisition
		IMAQdxStopAcquisition(session);
		}



	void HSLFilter(){
		for (int i = 0; i < (RES_X*RES_Y)/2; i=i+4){
			char R = raw_pixel[i];
			char B = raw_pixel[i+1];
			char G = raw_pixel[i+2];
			int hue = atan2( sqrt(3)*(G-B), (2*R)-G-B );

			//proc_pixel[(i/4)] = G;

		}

	}


	void DisabledInit()
	{
	}

	void DisabledPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
