#include "WPILib.h"
#include "NIIMAQdx.h"
#include <math.h>
#include "SimPID.h"
#include "SimLib.h"
#include "Gamepad.h"
#include <AHRS.h>

typedef unsigned long uInt32;
typedef long long int Int64;

#define RES_X 640
#define RES_Y 480
//1548, 3334
#define GP_UP 0
#define GP_DOWN 180
#define GP_A 1
#define GP_B 2
#define GP_X 3
#define GP_L 5
#define GP_R 6
#define OPEN 1
#define CLOSED 0

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

	SimPID *drivePID;
	SimPID *turnPID;
	SimPID *visionPID;

	Joystick *m_Gamepad;
	Joystick *m_Joystick;

	Timer *timer;
	double last_time;

	DigitalInput *m_intakeHomeSwitch;
	DigitalInput *m_pusherHomeSwitch;
	DigitalInput *m_boulderSwitch;

	Encoder *m_leftDriveEncoder;
	Encoder *m_rightDriveEncoder;

	AHRS *nav;

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
		m_shootE = new Solenoid(3);
		m_shootR = new Solenoid(2);

		m_Joystick = new Joystick(0);
		m_Gamepad = new Joystick(1);

		shooter1 = new CANTalon(0);
		shooter2 = new CANTalon(1);

		m_LED = new Relay(0);

		drivePID = new SimPID();
		drivePID->setMinDoneCycles(1);
		turnPID = new SimPID();
		turnPID->setMinDoneCycles(1);

		visionPID = new SimPID(0.01, 0, 0.005, 5);
		visionPID->setMaxOutput(0.5);
		visionPID->setMinDoneCycles(5);

		m_intakeHomeSwitch = new DigitalInput(1);
		m_boulderSwitch = new DigitalInput(0);

		m_leftDriveEncoder = new Encoder(4, 5);
		m_rightDriveEncoder = new Encoder(2, 3);

		m_intake = new CANTalon(2);
		/*
		m_intake->SetPID(1,0,0.4,1);
		m_intake->SetFeedbackDevice(CANTalon::QuadEncoder);
		m_intake->SetIzone(100);
		m_intake->SetCloseLoopRampRate(100);
		m_intake->SelectProfileSlot(0);
		m_intake->SetControlMode(CANTalon::kPosition);
		m_intake->SetClosedLoopOutputDirection(true);*/

		//nav = new AHRS(0);

		m_pusher = new CANTalon(3);
		/*
		m_pusher->SetPID(0,0,0,0);
		m_pusher->SetFeedbackDevice(CANTalon::QuadEncoder);
		m_pusher->SetIzone(100);
		m_pusher->SetCloseLoopRampRate(48);
		m_pusher->SelectProfileSlot(0);
		m_pusher->SetControlMode(CANTalon::kPosition);
		*/

		timer = new Timer();
		timer->Reset();

		shooterState = 0;


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
		aim_loop_counter = 0;

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
		//printf("intake enc position: %d\n", m_intake->GetEncPosition());
		//printf("pusher enc position: %d\n", m_pusher->GetEncPosition());
		if(m_Joystick->GetRawButton(10))
			m_intake->SetPosition(0);
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
		if(m_Joystick->GetRawButton(9))
			m_intake->SetPosition(0);
		m_LED->Set(Relay::kForward);
	}

	void TeleopPeriodic(void)
	{
		operateShifter();
		//simpleShoot();
		advancedShoot();
		pusher();
		//SubtractionFilter();
		//if (m_pusherHomeSwitch->Get())
			//m_pusher->Reset();
		//if (m_Joystick->GetRawButton(10))
			//aimAtTarget();
		if(!m_Gamepad->GetRawButton(GP_A)){
			FindTargetCenter();
			teleDrive();
		}
		//tempIntake();


		//lw->Run();
	}

	//==========================================================USER FUNCTIONS=================================
	inline void tempIntake(void){
		if(m_Gamepad->GetRawButton(GP_A)){
			shooter1->SetSetpoint(0.82/2);
			shooter2->SetSetpoint(-0.82/2);
		}
		else{
			shooter1->SetSetpoint(0.f);
			shooter2->SetSetpoint(0.f);
		}
	}

#define PRACTICE_DRIVE_LIMIT 0.65
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
#define SHOOT_SPEED 0.8225f
#define PICKUP 1700
#define SHOOT 650
#define HOME 0
	inline void simpleShoot(void){
		if (m_Joystick->GetRawButton(3)){
			shooter1->SetSetpoint(-SHOOT_SPEED);
			shooter2->SetSetpoint(SHOOT_SPEED);
		}
		else if (m_Joystick->GetRawButton(5)){
			shooter1->SetSetpoint(SHOOT_SPEED);
			shooter2->SetSetpoint(-SHOOT_SPEED);
		}
		else{
			shooter1->SetSetpoint(0.0);
			shooter2->SetSetpoint(0.0);
		}

#define SPEED 0.50
		printf("intake: %d error: %d\n", m_intake->GetEncPosition(), m_intake->GetClosedLoopError());
		if(m_Joystick->GetRawButton(10))
			m_intake->SetPosition(0);

		if(m_Joystick->GetRawButton(7)){
			m_intake->Set(PICKUP);
		}
		else if (m_Joystick->GetRawButton(8)){
			m_intake->Set(0);
		}
		else if(m_Joystick->GetRawButton(9))
			m_intake->Set(SHOOT);
		/*
		else{
			m_intake->SetSetpoint(0.0f);
			m_intake->SetEncPosition(0);
		}*/

		if (m_Joystick->GetRawButton(1)){
			m_shootE->Set(true);
			m_shootR->Set(false);
		}
		else{
			m_shootE->Set(false);
			m_shootR->Set(true);
		}
	}


#define PUSHER_SPEED 0.25
#define PUSHER_OUT 2170

	inline void pusher(void){
		if (m_Joystick->GetRawButton(6))
			m_pusher->SetSetpoint(PUSHER_SPEED);
		else if (m_Joystick->GetRawButton(4))
			m_pusher->SetSetpoint(-PUSHER_SPEED);
		else
			m_pusher->SetSetpoint(0.f);
	}

	void advancedShoot(void)
	{
		//printf("Shooter State: %d\n", shooterState);
		//printf("gamepad dY: %d\n", m_Gamepad->GetPOV(0));
		switch(shooterState)
		{
		case 0:
			//everything is off
			shooter1->SetSetpoint(0.f);
			shooter2->SetSetpoint(0.f);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_intakeHomeSwitch->Get())
				shooterState = 10;
			else
				shooterState = 20;
			break;
		case 10:
			//cylinder = extend|angle = home
			shooter1->SetSetpoint(0.f);
			shooter2->SetSetpoint(0.f);
			m_intake->Set(HOME);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			if(m_Gamepad->GetRawButton(GP_B))
				shooterState = 60;
			break;
		case 20:
			//cylinder = extend|angle = pickup
			shooter1->SetSetpoint(0.f);
			shooter2->SetSetpoint(0.f);
			m_intake->Set(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			else if(m_Gamepad->GetRawButton(GP_R))
				shooterState = 30;
			break;
		case 30:
			//cylinder = extend|angle = pickup|shooter = in
			shooter1->SetSetpoint(SHOOT_SPEED/2);
			shooter2->SetSetpoint(-SHOOT_SPEED/2);
			m_intake->Set(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			else if(m_Gamepad->GetRawButton(GP_L))
				shooterState = 20;
			else if(m_boulderSwitch->Get() == CLOSED)
				shooterState = 40;
			break;
		case 39:
			//cylinder = extend | angle = pickup | shooter out
			shooter1->SetSetpoint(-SHOOT_SPEED/2);
			shooter1->SetSetpoint(SHOOT_SPEED/2);
			m_intake->Set(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_boulderSwitch->Get() == OPEN)
				shooterState = 20;
			break;
		case 40:
			//cylinder = extend|angle = pickup
			shooter1->SetSetpoint(0.f);
			shooter2->SetSetpoint(0.f);
			m_intake->Set(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetRawButton(GP_L))
				shooterState = 20;
			else if(m_boulderSwitch->Get() == OPEN)
				shooterState = 30;
			else if(m_Gamepad->GetRawButton(GP_R))
				shooterState = 39;
			else if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 50;
			else if(m_Gamepad->GetRawButton(GP_B))
				shooterState = 60;
			break;
		case 50:
			m_shootE->Set(true);
			m_shootR->Set(false);
			m_intake->Set(HOME);
			shooter1->SetSetpoint(0.f);
			shooter2->SetSetpoint(0.f);
			//cylinder = extend|angle = home
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 40;
			else if(m_Gamepad->GetRawButton(GP_B))
				shooterState = 60;
			break;
		case 60:
			m_shootE->Set(true);
			m_shootR->Set(false);
			m_intake->Set(SHOOT);
			shooter1->SetSetpoint(0.f);
			shooter2->SetSetpoint(0.f);
			//cylinder = extend|angle = shoot
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 40;
			else if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 50;
			else if(true  && (m_Gamepad->GetRawButton(GP_A) || m_Gamepad->GetRawButton(GP_X))) //angle == shoot
			{
				shooterState = 61;
				timer->Start();
			}
			break;
		case 61:
			m_shootE->Set(false);
			m_shootR->Set(true);
			shooter1->SetSetpoint(SHOOT_SPEED/4);
			shooter2->SetSetpoint(-SHOOT_SPEED/4);
			m_intake->Set(SHOOT);
			if(timer->Get() > 0.25)
			{
				shooterState = 70;
				timer->Stop();
				timer->Reset();
			}
			break;
		case 69:
			m_shootE->Set(false);
			m_shootR->Set(true);
			shooter1->SetSetpoint(SHOOT_SPEED/4);
			shooter2->SetSetpoint(-SHOOT_SPEED/4);
			m_intake->Set(SHOOT);
			if(timer->Get() > 0.25)
			{
				shooterState = 60;
				timer->Stop();
				timer->Reset();
			}
			break;
		case 70:
			m_shootE->Set(false);
			m_shootR->Set(true);
			m_intake->Set(SHOOT);
			shooter1->SetSetpoint(-SHOOT_SPEED);
			shooter2->SetSetpoint(SHOOT_SPEED);
			//cylinder = retract|shooter = out|angle = shoot
			if(m_Gamepad->GetRawButton(GP_B))
			{
				timer->Start();
				shooterState = 69;
			}
			else if(m_Gamepad->GetRawButton(GP_X))
			{
				timer->Start();
				shooterState = 80;
			}
			else if(m_Gamepad->GetRawButton(GP_A))
				shooterState = 90;
			break;
		case 80:
			shooter1->SetSetpoint(-SHOOT_SPEED);
			shooter2->SetSetpoint(SHOOT_SPEED);
			m_intake->Set(SHOOT);
			m_shootE->Set(true);
			m_shootR->Set(false);
			//cylinder = extend|shooter = out|angle = shoot
			if(timer->Get() > 0.5)
			{
				shooterState = 10;
				timer->Stop();
				timer->Reset();
			}
			break;
		case 90:
			//vision
			if(!m_Gamepad->GetRawButton(GP_A))
				shooterState = 70;
			if(aimAtTarget() == 1){//goal object detected
				timer->Start();
				shooterState = 80;
			}
			break;
		}
	}

	bool autoDrive(int distance, int angle)
	{
		int currentDist = (m_rightDriveEncoder->Get() + m_leftDriveEncoder->Get()) / 2;
		int currentAngle = nav->GetYaw();

		drivePID->setDesiredValue(distance);
		turnPID->setDesiredValue(angle);

		float drive = drivePID->calcPID(currentDist);
		float turn = turnPID->calcPID(currentAngle);

		m_rightDrive2->SetSpeed(-(limit(drive - turn, 1)));
		m_rightDrive3->SetSpeed(-(limit(drive - turn, 1)));
		m_leftDrive4->SetSpeed(limit(drive + turn, 1));
		m_leftDrive1->SetSpeed(limit(drive + turn, 1));

		return drivePID->isDone() && turnPID->isDone();
	}

	//===============================================VISION FUNCTIONS=============================================
#define AIM_CORRECTION 80
#define AIM_FILTER 1
#define AIM_LOOP_WAIT 25
#define AIM_TIMEOUT 10

#define IMAGE_CENTER 320
	int aim_loop_counter;

	int aimAtTarget(void){
		float turn = last_turn;
		float error = -1000;
		int image_error;

		//take a picture after some time
		if(aim_loop_counter >= AIM_LOOP_WAIT){
			image_error = FindTargetCenter();
			//if the target was found calculate turn speed
			if (image_error == 0) {
				turnPID->setDesiredValue(IMAGE_CENTER - AIM_CORRECTION);
				turn = turnPID->calcPID(centerx);
				turn = AIM_FILTER*(turn - last_turn) + last_turn;

				last_turn = turn;
				aim_loop_counter = 0;
			}
			printf("im_error: %d\tcenter: %f\terror: %f\tturn: %f\n", image_error, centerx, error, turn);

			//SmartDashboard::PutNumber("Aim Error", IMAGE_CENTER + AIM_CORRECTION - x_pixel);
			//SmartDashboard::PutNumber("Motor Output", turn);
		}

		//if image hasn't been found in a while then stop moving
		if (aim_loop_counter - AIM_LOOP_WAIT >= AIM_TIMEOUT){
			turn = last_turn = 0;
		}

		aim_loop_counter++;

		m_leftDrive4->SetSpeed(turn);
		m_leftDrive1->SetSpeed(turn);
		m_rightDrive2->SetSpeed(turn);
		m_rightDrive3->SetSpeed(turn);

		if(visionPID->isDone()){
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

				//showBlobMeasurements();

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
			return 0;
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

#define R_THRESHOLD 150
#define G_THRESHOLD 100
#define B_THRESHOLD 150
	inline void BinaryFilter(void){
		for (int i = 0; i < (RES_X*RES_Y); i++){
			//printf("R, G, B: (%d, %d, %d)\n", R[i*4], G[i*4], B[i*4]);
			//if it has lots of red
			if ((R[i*4] > R_THRESHOLD)){
				proc_pixel[i] = 0;
			}
			//blue without green
			else if (((B[i*4] > B_THRESHOLD) && (G[i*4] < G_THRESHOLD))){
				//printf("eliminating pixel BG: %d, %d\n", B[i*4], G[i*4]);
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
