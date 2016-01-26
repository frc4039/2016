#include "WPILib.h"
#include <USBCamera.h>
#include <math.h>

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
	char proc_array[RES_Y][RES_X];
	char raw_array[RES_Y][RES_X*4];
	char *R, *G, *B;
	uInt32 numOfAttributes;
	IMAQdxAttributeInformation *cameraAttributes;
	uInt32 size;
	IMAQdxEnumItem *items;
	IMAQdxEnumItem exposure_mode;
	int exposure;
	int brightness;
	int contrast;
	int saturation;
	int cog_x, cog_y, num_of_pixels;



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


	void CameraSettings(void){
		//get all attributes
		IMAQdxEnumerateAttributes2(session, NULL, &numOfAttributes,"" , IMAQdxAttributeVisibilityAdvanced);
		printf("num of attributes: %d\n", (int)numOfAttributes);
		cameraAttributes = new IMAQdxAttributeInformation[numOfAttributes];
		IMAQdxEnumerateAttributes2(session, cameraAttributes, &numOfAttributes, "", IMAQdxAttributeVisibilityAdvanced);

		//get all values
		IMAQdxEnumerateAttributeValues(session, "CameraAttributes::Exposure::Mode", NULL, &size);
		printf("num of values: %d\n", (int)size);
		items = new IMAQdxEnumItem[size];
		IMAQdxEnumerateAttributeValues(session, "CameraAttributes::Exposure::Mode", items, &size);

/*
		for (unsigned int i = 0; i < numOfAttributes; i++)
			printf("Attribute %d (type %d, writable %d): %s\n", i, cameraAttributes[i].Type, cameraAttributes[i].Writable, cameraAttributes[i].Name);
		for (unsigned int i = 0; i < size; i++)
			printf("Name: %s\tValue: %d\tReserved: %d\d", items[i].Name, items[i].Value, items[i].Reserved);
*/
		IMAQdxGetAttribute(session, "CameraAttributes::Exposure::Mode", IMAQdxValueTypeEnumItem, &exposure_mode);
		exposure_mode.Value = (uInt32)1;

		printf("Imaq error exposure mode: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Exposure::Mode", IMAQdxValueTypeEnumItem, exposure_mode));
		printf("Imaq error exposure value: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Exposure::Value", IMAQdxValueTypeI64, (Int64)10));
		printf("Imaq error brightness: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Brightness::Value", IMAQdxValueTypeI64, (Int64)220));
		printf("Imaq error contrast: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Contrast::Value", IMAQdxValueTypeI64, (Int64)10));
		//printf("Imaq error saturation: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Saturation::Value", IMAQdxValueTypeI64, (Int64)40));

		//check the settings
		IMAQdxGetAttribute(session, "CameraAttributes::Exposure::Mode", IMAQdxValueTypeEnumItem, &exposure_mode);
		IMAQdxGetAttribute(session, "CameraAttributes::Exposure::Value", IMAQdxValueTypeI64, &exposure);
		IMAQdxGetAttribute(session, "CameraAttributes::Brightness::Value", IMAQdxValueTypeI64, &brightness);
		IMAQdxGetAttribute(session, "CameraAttributes::Contrast::Value", IMAQdxValueTypeI64, &contrast);
		IMAQdxGetAttribute(session, "CameraAttributes::Saturation::Value", IMAQdxValueTypeI64, &saturation);


		printf("exposure (mode %s(%d)): %d\n", exposure_mode.Name, (int)exposure_mode.Value, (int)exposure);
		printf("brightness: %d\n", (int)brightness);
		printf("contrast: %d\n", (int)contrast);
		printf("saturation: %d\n", (int)saturation);
	}

	void VisionInit(void){

		//Camera = new USBCamera("cam0", false);

		//initialize image data structure (no size)
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		processed = imaqCreateImage(ImageType::IMAQ_IMAGE_U8,0);

		//initialize image with a size
		imaqArrayToImage(frame, &raw_array, RES_X, RES_Y);
		imaqArrayToImage(processed, &proc_array, RES_X, RES_Y);

		//get image pointers and info
		imaqGetImageInfo(frame, &raw_info);
		imaqGetImageInfo(processed, &proc_info);

		//set up pixel pointers
		raw_pixel = (char*)(raw_info.imageStart);
		proc_pixel = (char*)(proc_info.imageStart);
		R = raw_pixel;
		G = raw_pixel + 1;
		B = raw_pixel + 2;



		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
		CameraSettings();

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

			if(imaqError != IMAQdxErrorSuccess)
			{
				DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
			}
			else
			{

				HSLFilter();

				CameraServer::GetInstance()->SetImage(processed);

			}

			Wait(0.005);				// wait for a motor update time
		}

		// stop image acquisition
		IMAQdxStopAcquisition(session);
		}


#define THRESHOLD 40
#define CIRCLE_SIZE 100

	void HSLFilter(){
		//also find COG here and maybe test for u shape
		cog_x = cog_y = num_of_pixels = 0;

		for (int i = 0; i < (RES_X*RES_Y); i++){
			//int hue = atan2( sqrt(3)*(G[i*4]-B[i*4]), (2*R[i*4])-G[i*4]-B[i*4] );


			if (G[i*4] > THRESHOLD){
				proc_pixel[i] = 255;
				cog_y += i % RES_X;
				cog_x += (int)(i / RES_X);
				num_of_pixels++;
				//printf("cog: (%d,%d) # %d\n", cog_x, cog_y);
				//Wait(0.1);
			}
			else
				proc_pixel[i] = 0;


			//proc_pixel[i] = G[i*4];
		}
		cog_x = cog_x / num_of_pixels;
		cog_y = cog_y / num_of_pixels;
		//printf("final cog: (%d,%d) # %d\n", cog_x, cog_y, num_of_pixels);
		if (num_of_pixels > 500)
			imaqDrawShapeOnImage(processed, processed, {cog_x-100,cog_y-100,200,200}, IMAQ_DRAW_INVERT,IMAQ_SHAPE_OVAL,255);

	}


	void DisabledInit()
	{
	}

	void DisabledPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
