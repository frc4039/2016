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

	Victor *m_leftDrive4; //0
	Victor *m_leftDrive1; //1
	Victor *m_rightDrive2; //2
	Victor *m_rightDrive3; //3
	CANTalon *talon;

	Joystick *m_Joystick;
	Timer *timer;
	double last_time;

	//=======================Vision Variables======================
	USBCamera *Camera;
	IMAQdxSession session;

	Image *frame;
	Image *processed;
	Image *particle;
	ImageInfo raw_info;
	ImageInfo proc_info;
	IMAQdxError imaqError;
	char *raw_pixel;
	char *proc_pixel;
	char proc_array[RES_Y][RES_X];
	char raw_array[RES_Y][RES_X*4];
	char *R, *G, *B, *alpha;
	RGBValue colourTable;
	uInt32 numOfAttributes;
	IMAQdxAttributeInformation *cameraAttributes;
	uInt32 size;
	IMAQdxEnumItem *items;
	IMAQdxEnumItem exposure_mode;
	int exposure;
	int brightness;
	int contrast;
	int saturation;
	float64 Gr, Gb, Gg;
	int cog_x, cog_y, num_of_pixels;
	int picture_ID;

	//vision filter options
	ParticleFilterOptions2 filterOptions;
#define CRITERIA_COUNT 	1
	ParticleFilterCriteria2 filterCriteria[CRITERIA_COUNT];
	int num_particlesFound;
	MeasurementType measurements[1];
	ROI *roi;



	void RobotInit() override
	{
		m_leftDrive4 = new Victor(4);
		m_leftDrive1 = new Victor(1);
		m_rightDrive2 = new Victor(2);
		m_rightDrive3 = new Victor(3);

		//talon = new CANTalon(1);

		m_Joystick = new Joystick(0);

		VisionInit();
	}

#define EXPOSURE (Int64)10
#define BRIGHTNESS (Int64)150
#define CONTRAST (Int64)10
#define SATURATION (Int64)100
#define R_GAIN (float64)0
#define G_GAIN (float64)1
#define B_GAIN (float64)0

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

		//set the settings
		printf("Imaq error exposure mode: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Exposure::Mode", IMAQdxValueTypeEnumItem, exposure_mode));
		printf("Imaq error exposure value: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Exposure::Value", IMAQdxValueTypeI64, EXPOSURE));
		printf("Imaq error brightness: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Brightness::Value", IMAQdxValueTypeI64, BRIGHTNESS));
		printf("Imaq error contrast: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Contrast::Value", IMAQdxValueTypeI64, CONTRAST));
		printf("Imaq error saturation: %d\n", IMAQdxSetAttribute(session, "CameraAttributes::Saturation::Value", IMAQdxValueTypeI64, SATURATION));
		printf("Imaq error Gain R: %d\n", IMAQdxSetAttribute(session, "AcquisitionAttributes::Bayer::GainR", IMAQdxValueTypeF64, R_GAIN));
		printf("Imaq error Gain R: %d\n", IMAQdxSetAttribute(session, "AcquisitionAttributes::Bayer::GainG", IMAQdxValueTypeF64, G_GAIN));
		printf("Imaq error Gain R: %d\n", IMAQdxSetAttribute(session, "AcquisitionAttributes::Bayer::GainB", IMAQdxValueTypeF64, B_GAIN));

		//check the settings
		IMAQdxGetAttribute(session, "CameraAttributes::Exposure::Mode", IMAQdxValueTypeEnumItem, &exposure_mode);
		IMAQdxGetAttribute(session, "CameraAttributes::Exposure::Value", IMAQdxValueTypeI64, &exposure);
		IMAQdxGetAttribute(session, "CameraAttributes::Brightness::Value", IMAQdxValueTypeI64, &brightness);
		IMAQdxGetAttribute(session, "CameraAttributes::Contrast::Value", IMAQdxValueTypeI64, &contrast);
		IMAQdxGetAttribute(session, "CameraAttributes::Saturation::Value", IMAQdxValueTypeI64, &saturation);
		IMAQdxGetAttribute(session, "AcquisitionAttributes::Bayer::GainB", IMAQdxValueTypeF64, &Gb);
		IMAQdxGetAttribute(session, "AcquisitionAttributes::Bayer::GainR", IMAQdxValueTypeF64, &Gr);
		IMAQdxGetAttribute(session, "AcquisitionAttributes::Bayer::GainG", IMAQdxValueTypeF64, &Gg);
		printf("RGB gain: %f, %f, %f\n", Gr, Gg, Gb);
		printf("exposure (mode %s(%d)): %d\n", exposure_mode.Name, (int)exposure_mode.Value, (int)exposure);
		printf("brightness: %d\n", (int)brightness);
		printf("contrast: %d\n", (int)contrast);
		printf("saturation: %d\n", (int)saturation);
	}

	void ParticleFilterInit(void){
		//options config
		roi = imaqCreateROI();
		imaqAddRectContour(roi, imaqMakeRect(0, 0, RES_X, RES_Y));
		filterOptions.connectivity8 = FALSE;
		filterOptions.fillHoles = FALSE;
		filterOptions.rejectBorder = FALSE;
		filterOptions.rejectMatches = FALSE;
		measurements[0] = IMAQ_MT_AREA;

		//common filter config
		for (int i = 0; i < CRITERIA_COUNT; i++){
			filterCriteria[i].calibrated = FALSE;
			filterCriteria[i].exclude = FALSE;
		}

		//area config
		filterCriteria[0].parameter = IMAQ_MT_AREA;
		filterCriteria[0].lower = 500;
		filterCriteria[0].upper = 1000000;



		/*
		//width config
		filterCriteria[1].parameter = IMAQ_MT_BOUNDING_RECT_WIDTH;
		filterCriteria[1].lower = 1;
		filterCriteria[1].upper = 100000;

		//height config
		filterCriteria[2].parameter = IMAQ_MT_BOUNDING_RECT_HEIGHT;
		filterCriteria[2].lower = 1;
		filterCriteria[2].upper = 100000;*/
}

	void VisionInit(void){

		//initialize image data structure (no size)
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		processed = imaqCreateImage(ImageType::IMAQ_IMAGE_U8,0);
		particle = imaqCreateImage(ImageType::IMAQ_IMAGE_U8,0);


		//initialize image with a size
		imaqArrayToImage(frame, &raw_array, RES_X, RES_Y);
		imaqArrayToImage(processed, &proc_array, RES_X, RES_Y);
		imaqArrayToImage(particle, &proc_array, RES_X, RES_Y);

		//get image pointers and info
		imaqGetImageInfo(frame, &raw_info);
		imaqGetImageInfo(processed, &proc_info);

		//set up pixel pointers
		raw_pixel = (char*)(raw_info.imageStart);
		proc_pixel = (char*)(proc_info.imageStart);
		R = raw_pixel;
		G = raw_pixel + 1;
		B = raw_pixel + 2;
		alpha = raw_pixel + 3;

		colourTable = {255,255,255,255};
		picture_ID = 0;

		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
		CameraSettings();
		ParticleFilterInit();

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

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		IMAQdxStartAcquisition(session);

	}

	void TeleopPeriodic()
	{
		//teleDrive();
		//if (m_Joystick->GetRawButton(10))
			TakePicture();

		//lw->Run();
	}

	inline void teleDrive()
	{
		float leftSpeed = limit(expo(m_Joystick->GetY(), 2), 1) - limit(expo(m_Joystick->GetX(), 5), 1);
		float rightSpeed = -limit(expo(m_Joystick->GetY(), 2), 1) - limit(expo(m_Joystick->GetX(), 5), 1);

		//printf("Joystick x=%f, y=%f\n", x,y);
		m_leftDrive4->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);
	}

	void TakePicture()
	{
		// acquire images

		IMAQdxGrab(session, frame, true, NULL);

		if(imaqError != IMAQdxErrorSuccess)
		{
			DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
		}
		else
		{
			//filter image for blob finding
			HSLFilter();

			//find blobs filtered
			imaqParticleFilter4(particle, processed, filterCriteria, CRITERIA_COUNT, &filterOptions, NULL, &num_particlesFound);
			//printf("error: %d\tparticles found: %d\n", error, num_particlesFound);

			double rect_left, rect_width, center_mass_y;
			if (num_particlesFound > 1)
				DriverStation::ReportError("Warning! Multiple blobs found!");
			for (int i = 0; i < num_particlesFound; i++)
			{
				imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &rect_left);
				imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &rect_width);
				imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_CENTER_OF_MASS_Y, &center_mass_y);
				//imaqMeasureParticle(processed, i, FALSE, IMAQ_MT_CENTER_OF_MASS_Y, &resulty);
				//printf("%d blob area: %f\n", particleReport, resultx);
			}
			double centerx = rect_left + (rect_width/2);
			imaqDrawShapeOnImage(processed, processed, {center_mass_y - (rect_width/2), centerx - (rect_width/2), rect_width, rect_width}, IMAQ_DRAW_INVERT,IMAQ_SHAPE_OVAL,255);


			if (m_Joystick->GetRawButton(11))
				CameraServer::GetInstance()->SetImage(frame);
			else if (m_Joystick->GetRawButton(9))
				CameraServer::GetInstance()->SetImage(particle);
			else
				CameraServer::GetInstance()->SetImage(processed);



			if(m_Joystick->GetRawButton(12)){
				char *filename;
				sprintf(filename, "/home/lvuser/pic%d.bmp", picture_ID);
				DriverStation::ReportError("writing picture to file\n");
				imaqWriteBMPFile(frame, filename, 30, &colourTable);
			}

		}

		// stop image acquisition
	}


#define CIRCLE_SIZE 100
#define SIMILARITY 10
#define R_THRESHOLD 100
#define G_THRESHOLD 70
#define B_THRESHOLD 100


	void HSLFilter(){
		//also find COG here and maybe test for u shape
		cog_x = cog_y = num_of_pixels = 0;

		for (int i = 0; i < (RES_X*RES_Y); i++){
			//int hue = atan2( sqrt(3)*(G[i*4]-B[i*4]), (2*R[i*4])-G[i*4]-B[i*4] );


			if ((R[i*4] > R_THRESHOLD) || ((B[i*4] > B_THRESHOLD) && (G[i*4] < G_THRESHOLD))){
				proc_pixel[i] = 0;
				//printf("cog: (%d,%d) # %d\n", cog_x, cog_y);
				}
			else if (G[i*4] > G_THRESHOLD){
				proc_pixel[i] = 255;
				cog_y += i % RES_X;
				cog_x += (int)(i / RES_X);
				num_of_pixels++;
			}
			else
				proc_pixel[i] = 0;

			//proc_pixel[i] = alpha[i*4];
		}
		cog_x = cog_x / num_of_pixels;
		cog_y = cog_y / num_of_pixels;
		//printf("final cog: (%d,%d) # %d\n", cog_x, cog_y, num_of_pixels);
		//if (num_of_pixels > 500)
			//imaqDrawShapeOnImage(processed, processed, {cog_x-100,cog_y-100,200,200}, IMAQ_DRAW_INVERT,IMAQ_SHAPE_OVAL,255);


	}

	float expo(float x, int n)
	{
		int sign = n % 2;
		float y = 1;
		for (int i = 1; i <= n; i++)
		{
			y *= x;
		}
		if(sign == 0 && x < 0)
			return -y;
		return y;
	}

	float limit(float x, float lim)
	{
		if (x > lim)
			return lim;
		else if (x < -lim)
			return -lim;
		return x;
	}

	void DisabledInit()
	{
		IMAQdxStopAcquisition(session);
	}

	void DisabledPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
