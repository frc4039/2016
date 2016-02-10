#include "WPILib.h"
#include <math.h>
//#include <AHRS.h>

#define RES_X 640
#define RES_Y 480

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();

	int autoState, autoMode, shooterState;

	Victor *m_leftDrive4; //0
	Victor *m_leftDrive1; //1
	Victor *m_rightDrive2; //2
	Victor *m_rightDrive3; //3
	float leftSpeed, rightSpeed;
	CANTalon *shooter1, *shooter2, *m_intake, *m_pusher;
	Relay *m_LED;

	Solenoid *m_shiftHigh, *m_shiftLow;
	Solenoid *m_shootE, *m_shootR;

	Joystick *m_Joystick;
	Timer *timer;
	double last_time;

	//=======================Vision Variables======================
	IMAQdxSession session;
	Image *frame;
	Image *subtracted;
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
	int picture_ID;
	char filename[25];
	double rect_left, rect_width, center_mass_y, centerx, last_turn;


	//vision filter options
	ParticleFilterOptions2 filterOptions;
#define CRITERIA_COUNT 3
	ParticleFilterCriteria2 filterCriteria[CRITERIA_COUNT];
	int num_particlesFound;
	MeasurementType measurements[1];

//====================================================INIT==============================================
	void RobotInit(void) override
	{
		m_leftDrive4 = new Victor(4);
		m_leftDrive1 = new Victor(1);
		m_rightDrive2 = new Victor(2);
		m_rightDrive3 = new Victor(3);

		//talon = new CANTalon(1);
		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
		m_shootE = new Solenoid(2);
		m_shootR = new Solenoid(3);

		m_Joystick = new Joystick(0);

		shooter1 = new CANTalon(0);
		shooter2 = new CANTalon(1);
		m_intake = new CANTalon(2);
		m_pusher = new CANTalon(3);

		m_LED = new Relay(0);

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
		filterOptions.connectivity8 = FALSE;
		filterOptions.fillHoles = TRUE;
		filterOptions.rejectBorder = FALSE;
		filterOptions.rejectMatches = FALSE;

		//common filter config
		for (int i = 0; i < CRITERIA_COUNT; i++){
			filterCriteria[i].calibrated = FALSE;
			filterCriteria[i].exclude = FALSE;
		}

		//area config
		filterCriteria[0].parameter = IMAQ_MT_AREA;
		filterCriteria[0].lower = 500;
		filterCriteria[0].upper = 1000000;

		//width config
		filterCriteria[1].parameter = IMAQ_MT_BOUNDING_RECT_WIDTH;
		filterCriteria[1].lower = 60;
		filterCriteria[1].upper = 300;

		//height config
		filterCriteria[2].parameter = IMAQ_MT_BOUNDING_RECT_HEIGHT;
		filterCriteria[2].lower = 50;
		filterCriteria[2].upper = 300;

		//add perimeter filter
		//filterCriteria[3].parameter = IMAQ_MT_PERIMETER;
		//filterCriteria[3].lower = 50;
		//filterCriteria[3].upper = 100;
}

	void VisionInit(void){

		//initialize image data structure (no size)
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		subtracted = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		processed = imaqCreateImage(ImageType::IMAQ_IMAGE_U8,0);
		particle = imaqCreateImage(ImageType::IMAQ_IMAGE_U8,0);

		//initialize image with a size
		imaqArrayToImage(frame, &raw_array, RES_X, RES_Y);
		imaqArrayToImage(subtracted, &raw_array, RES_X, RES_Y);
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

		//misc set up
		colourTable = {255,255,255,255};
		picture_ID = 0;
		last_turn = 0;

		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);

		//initialize capture settings and filters
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

	//======================================END OF INIT=======================================================
	void DisabledInit()
	{
		IMAQdxStopAcquisition(session);
		m_LED->Set(Relay::kOff);
	}

	void DisabledPeriodic()
	{
		//FindTargetCenter();
	}

	//========================================================AUTONOMOUS=======================================
	void AutonomousInit(void)
	{

	}

	void AutonomousPeriodic(void)
	{

	}

	//===========================================================TELEOP=======================================
	void TeleopInit(void)
	{
		IMAQdxStartAcquisition(session);
		m_LED->Set(Relay::kForward);
	}

	void TeleopPeriodic(void)
	{
		teleDrive();
		operateShifter();
		simpleShoot();
		pusher();
		SubtractionFilter();
		//if (m_Joystick->GetRawButton(10))
			//aimAtTarget();
		//FindTargetCenter();

		//lw->Run();
	}

	//==========================================================USER FUNCTIONS=================================
#define PRACTICE_DRIVE_LIMIT 0.35
	inline void teleDrive(void)
	{
		leftSpeed = scale(limit(expo(m_Joystick->GetY(), 2), 1) - scale(limit(expo(m_Joystick->GetX(), 5), 1), 0.75f), PRACTICE_DRIVE_LIMIT);
		rightSpeed = scale(-limit(expo(m_Joystick->GetY(), 2), 1) - scale(limit(expo(m_Joystick->GetX(), 5), 1), 0.75f), PRACTICE_DRIVE_LIMIT);

		//printf("Joystick x=%f, y=%f\n", x,y);
		m_leftDrive4->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);
	}

	inline void operateShifter(void){
		if(m_Joystick->GetRawButton(2)){
			m_shiftHigh->Set(true);
			m_shiftLow->Set(false);
		}
		else{
			m_shiftHigh->Set(false);
			m_shiftLow->Set(true);
		}
	}
#define SIMPLE_SHOOT_SPEED 0.8225f
	inline void simpleShoot(void){
		if (m_Joystick->GetRawButton(3)){
			shooter1->SetSetpoint(-SIMPLE_SHOOT_SPEED);
			shooter2->SetSetpoint(SIMPLE_SHOOT_SPEED);
		}
		else if (m_Joystick->GetRawButton(5)){
			shooter1->SetSetpoint(SIMPLE_SHOOT_SPEED);
			shooter2->SetSetpoint(-SIMPLE_SHOOT_SPEED);
		}
		else{
			shooter1->SetSetpoint(0.0);
			shooter2->SetSetpoint(0.0);
		}

#define SPEED 0.50
		if(m_Joystick->GetRawButton(7)){
			m_intake->SetSetpoint(SPEED);
		}
		else if (m_Joystick->GetRawButton(8)){
			m_intake->SetSetpoint(-SPEED);
		}
		else{
			m_intake->SetSetpoint(0.0f);
		}

		if (m_Joystick->GetRawButton(1)){
			m_shootE->Set(true);
			m_shootR->Set(false);
		}
		else{
			m_shootE->Set(false);
			m_shootR->Set(true);
		}
	}


/*	void advancedShoot()
	{
		switch(shooterState)
	}*/

#define PUSHER_SPEED 0.25
	inline void pusher(void){
		if (m_Joystick->GetRawButton(6))
			m_pusher->SetSetpoint(PUSHER_SPEED);
		else if (m_Joystick->GetRawButton(4))
			m_pusher->SetSetpoint(-PUSHER_SPEED);
		else
			m_pusher->SetSetpoint(0.f);
	}
	//===============================================VISION FUNCTIONS=============================================
#define AIM_P 0.003f
#define AIM_E 10
#define AIM_LIM 0.5
#define AIM_CORRECTION 0
#define AIM_FILTER 0.5

#define IMAGE_CENTER 320
	int aimAtTarget(void){
		int x_pixel = FindTargetCenter();
		float turn;
		float error = IMAGE_CENTER + AIM_CORRECTION - x_pixel;
		if(x_pixel > 0){
			turn = limit(AIM_P * error, AIM_LIM);
			turn = AIM_FILTER*(turn - last_turn) + last_turn;
			last_turn = turn;

			//SmartDashboard::PutNumber("Aim Error", IMAGE_CENTER + AIM_CORRECTION - x_pixel);
			//SmartDashboard::PutNumber("Motor Output", turn);
		}
		else
			turn = 0;

		m_leftDrive4->SetSpeed(turn);
		m_leftDrive1->SetSpeed(turn);
		m_rightDrive2->SetSpeed(turn);
		m_rightDrive3->SetSpeed(turn);

		if((error) < AIM_E && (error) > -AIM_E){
			//printf("SHOOT THE BALL!!!\n");
			return 1;
		}
		//printf("Do not shoot yet.\n");
		return 0;
	}

	int FindTargetCenter(void)
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
			BinaryFilter();

			//find filtered blobs
			imaqParticleFilter4(particle, processed, filterCriteria, CRITERIA_COUNT, &filterOptions, NULL, &num_particlesFound);

			if (num_particlesFound > 1){
				DriverStation::ReportError("ERROR! Multiple blobs found!\n");
				CameraServer::GetInstance()->SetImage(processed);
				//unsure which blob is target
				return -2;
			}
			else if (num_particlesFound == 0){
				//unable to find target
				DriverStation::ReportError("ERROR! Target not found!\n");
				CameraServer::GetInstance()->SetImage(frame);
				return -3;
			}
			else if (num_particlesFound == 1){
				//take measurements
				imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &rect_left);
				imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &rect_width);
				imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_CENTER_OF_MASS_Y, &center_mass_y);

				showBlobMeasurements();

				//find center based on width
				centerx = rect_left + (rect_width/2);

				//optional draw circle, or reference lines for visual confirmation
				//imaqDrawShapeOnImage(processed, processed, {(int)(center_mass_y - (rect_width/2.f)), (int)(centerx - (rect_width/2.f)), (int)rect_width, (int)rect_width}, IMAQ_DRAW_INVERT,IMAQ_SHAPE_OVAL,0);
				imaqDrawShapeOnImage(processed, processed, {0, IMAGE_CENTER+AIM_CORRECTION, RES_Y-1, 1}, IMAQ_DRAW_INVERT, IMAQ_SHAPE_RECT, 0);
				imaqDrawShapeOnImage(processed, processed, {(int)(center_mass_y - rect_width/2.f), (int)centerx, (int)(rect_width/2), 1}, IMAQ_DRAW_INVERT, IMAQ_SHAPE_RECT, 0);
				SmartDashboard::PutNumber("target center", centerx);
				//printf("target width, x, y: %f\t%f\t%f\n", rect_width, centerx, center_mass_y);
			}

			if (m_Joystick->GetRawButton(11))
				CameraServer::GetInstance()->SetImage(frame);
			else
				CameraServer::GetInstance()->SetImage(processed);

			if(m_Joystick->GetRawButton(12)){
				sprintf(filename, "/home/lvuser/pic%d.bmp", picture_ID);
				DriverStation::ReportError("writing picture to file\n");
				imaqWriteBMPFile(processed, filename, 30, &colourTable);
			}

			return centerx;
		}
		// unable to take picture, return error
		DriverStation::ReportError("ERROR! Unable to take picture!\n");
		return -1;
	}

	void showBlobMeasurements(void){
		double width, left, area, perimeter;
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &width);
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &left);
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_AREA, &area);
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_PERIMETER, &perimeter);

		printf("width: %d\tleft: %d\tarea: %d\tperimeter: %d\n", (int)width, (int)left, (int)area, (int)perimeter);
	}

#define R_THRESHOLD 100
#define G_THRESHOLD 60
#define B_THRESHOLD 100
	inline void BinaryFilter(void){
		for (int i = 0; i < (RES_X*RES_Y); i++){
			//if it has lots of red
			if ((R[i*4] > R_THRESHOLD)){
				proc_pixel[i] = 0;
			}
			//blue without green
			else if (((B[i*4] > B_THRESHOLD) && (G[i*4] < G_THRESHOLD))){
				printf("eliminating pixel BG: %d, %d\n", B[i*4], G[i*4]);
				proc_pixel[i] = 0;
			}
			//if lots of green
			else if ((G[i*4] > G_THRESHOLD)){
				//printf("got green pixel: %d\n", G[i*4]);
				proc_pixel[i] = 255;
			}
			else
				proc_pixel[i] = 0;
		}
	}

	inline void SubtractionFilter(void){
		//take pics with light on and off
		IMAQdxGrab(session, frame, true, NULL);
		m_LED->Set(Relay::kOff);
		IMAQdxGrab(session, subtracted, true, NULL);
		m_LED->Set(Relay::kForward);


		if (m_Joystick->GetRawButton(11))
			CameraServer::GetInstance()->SetImage(frame);
		else if (m_Joystick->GetRawButton(10))
			CameraServer::GetInstance()->SetImage(subtracted);
		else{
			imaqSubtract(subtracted, frame, subtracted);
			imaqDrawShapeOnImage(subtracted, subtracted, {0, IMAGE_CENTER+AIM_CORRECTION, RES_Y-1, 1}, IMAQ_DRAW_INVERT, IMAQ_SHAPE_RECT, 0);
			CameraServer::GetInstance()->SetImage(subtracted);
		}

	}

	//=============================================MATHY FUNCTIONS=======================================
	inline float expo(float x, int n)
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

	inline float scale(float x, float scale){
		return x * scale;
	}

	inline float limit(float x, float lim)
	{
		if (x > lim)
			return lim;
		else if (x < -lim)
			return -lim;
		return x;
	}

};

START_ROBOT_CLASS(Robot)
