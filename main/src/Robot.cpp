/**
 * Changelog: please comment your commit messages
 * April 1- BTK
 * In all shoot autos, if robot taking too long to align, will shoot if we're within 3 pixels of target
 * Trimmed PID and minoutput for turnPID2
 * Increased auto_over_other distance
 *
 * April 5- LW
 * Changed autos to drive backward
 * Added portcullis auto (untested)
 * Added CoF auto (untested)
 * If any of you mess up the tabs again I swear to god I will create my own branch
 * April 13- BTK
 * Trimmed auto distance constants
 *
 * April 14
 * Fixed up auto modes, added portcullis distance constant, added operateProtectiveDevice function (untested)
 *
 * April 21- LW
 * Added code for PD Servos, functions, variables, constants
 */

#include "WPILib.h"
#include "NIIMAQdx.h"
#include <math.h>
#include "SimPID.h"
#include "AHRS.h"



//gamepad button locations
#define JS_RIGHT 90
#define JS_LEFT 270
#define GP_UP 0
#define GP_DOWN 180
#define GP_LEFT 270
#define GP_RIGHT 90
#define GP_A 1
#define GP_B 2
#define GP_X 3
#define GP_Y 4
#define GP_L 5
#define GP_R 6
#define GP_SELECT 7
#define GP_START 8

//servo and sensor constants
#define OPEN 1
#define CLOSED 0
#define SERVO_IN_1 0
#define SERVO_OUT_1 90
#define SERVO_IN_2 0
#define SERVO_OUT_2 90

//current shooter constants
//#define SPEED_RPM 6500
#define SPEED_RPM 1600
#define SPEED_RPM_LOW 1100
//#define SPEED_RPM_LOW 3900
#define BALL_SPIN ((int)(SPEED_RPM*0.3))

//legacy shooter constants
#define SHOOT_SPEED 0.8225f
#define SHOOTER_SPEED_CHECK 20000

//appendage constants
#define PICKUP 1800
#define SHOOT_LOWBAR 390
#define SHOOT_FAR 620
#define SHOOT_CLOSE 665
#define INTAKE_SHOOT_FAR 650
#define INTAKE_SHOOT_CLOSE 700
#define TRANSFER 0
#define HOME_SHOOTER 0
#define HOME_INTAKE 0
#define PUSHER_OUT 2170

//manual aim constants
#define MAN_AIM_ON 0.005
#define MAN_AIM_OFF 1.0
#define MAN_AIM_SPEED 0.3

//miscellaneous constants
#define PUSHER_SPEED 0.25
#define ROLLER_SPEED -0.5
#define PRACTICE_DRIVE_LIMIT 1
#define PI 3.141592653589793f

//autonomous constants
#define AUTO_OVER_MOAT -17000
#define AUTO_OVER_OTHER -16000
#define AUTO_OVER_ROUGH -14000
#define AUTO_LOWBAR_DRIVE -18000
#define AUTO_CHEVAL_DRIVE_1 5100
#define AUTO_CHEVAL_DRIVE_2 14000
#define AUTO_PORTCULLIS_DRIVE_1 5500
#define AUTO_PORTCULLIS_DRIVE_2 14000
#define NEUTRAL_DRIVE -3000
#define AUTO_AIM_POS_1 45
#define AUTO_AIM_POS_2_L -25
#define AUTO_AIM_POS_2_R 35
#define AUTO_AIM_POS_3 0
#define AUTO_AIM_POS_4 5
#define AUTO_AIM_POS_5 -15
#define AUTO_SHOOTER_POS SHOOT_FAR+40
#define AUTO_LOWBAR_ANGLE 45

//VISION SETTINGS, READ CAREFULLY
//left right trim for robot aim in pixels
//try this first if change is needed
#define AIM_CORRECTION 30
//tells robot to save the pictures it takes when trying to shoot
//comment out to not save pictures
#define SAVE_SHOT_PICTURES

//camera settings, DO NOT TOUCH
#define EXPOSURE (Int64)10
#define BRIGHTNESS (Int64)150
#define CONTRAST (Int64)10
#define SATURATION (Int64)100
#define R_GAIN (float64)0
#define G_GAIN (float64)1
#define B_GAIN (float64)0
#define IMAGE_CENTER 320
#define CRITERIA_COUNT 4
#define RES_X 640
#define RES_Y 480

//filter settings, VERIFY WITH MATLAB DO NOT GUESS
#define R_THRESHOLD 125
#define G_THRESHOLD 180
#define B_THRESHOLD 200

//legacy constants, don't bother
#define AIM_FILTER 1
#define AIM_LOOP_WAIT 5
#define AIM_TIMEOUT 2
#define AIM_FINE_LIMIT 20
#define CLOSE_LIMIT 220

//for vertical aim calculations, you can change trim
//but don't touch others unless verified with excel sheet
#define SLOPE -0.0779f
#define INTERCEPT 58.699f/
#define SHOOTER_TRIM 1.f

//for horizontal aim, you can change aim correction
#define HFOV 0.449422282f //found through experimentation, DO NOT TOUCH
#define AUTO_AIM_CORRECTION 0.5

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();

	int autoState, autoMode, autoPosition, shooterState, pusherState, shooterState1, autoDirection, servoState;
	int pastLeft, pastRight;

	VictorSP *m_leftDrive4; //4
	VictorSP *m_leftDrive1; //1
	VictorSP *m_rightDrive2; //2
	VictorSP *m_rightDrive3; //3
	float leftSpeed, rightSpeed, winchSpeed, time, motorSpeed;

	VictorSP *m_climber;
	VictorSP *m_intakeRoller;

	CANTalon *shooter1, *shooter2, *m_shooter, *m_intake;

	PowerDistributionPanel *PDP;

	Relay *m_LED;

	Solenoid *m_shiftHigh, *m_shiftLow;
	Solenoid *m_shootE, *m_shootR;
	Solenoid *m_climbE, *m_climbR;

	Servo *m_PDServo1;
	Servo *m_PDServo2;

	SimPID *drivePID;
	SimPID *turnPID;
	SimPID *visionPID;
	SimPID *intakePID;
	SimPID *shooterPID;
	SimPID *turnPID2;

	Joystick *m_Gamepad;
	Joystick *m_Gamepad2;
	Joystick *m_Joystick;

	Timer *timer;
	Timer *trimTimer;
	Timer *stateTimer;
	Timer *pressTimer;
	double last_time;

	DigitalInput *m_shooterHomeSwitch;
	DigitalInput *m_intakeHomeSwitch;
	DigitalInput *m_pusherHomeSwitch;

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
	double rect_left, rect_width, target_y, centerx, last_turn;
	float autoDelay;
	float autoAimAngle;
	int aim_loop_counter;
	float aim_fly_trim;


	//vision filter options
	ParticleFilterOptions2 filterOptions;
	ParticleFilterCriteria2 filterCriteria[CRITERIA_COUNT];
	int num_particlesFound;
	MeasurementType measurements[1];

//====================================================INIT==============================================
	void RobotInit(void) override

	{
		m_leftDrive4 = new VictorSP(4);
		m_leftDrive1 = new VictorSP(1);
		m_rightDrive2 = new VictorSP(2);
		m_rightDrive3 = new VictorSP(3);

		m_shiftHigh = new Solenoid(1);
		m_shiftLow = new Solenoid(0);
		m_shootE = new Solenoid(2);
		m_shootR = new Solenoid(3);
		m_climbE = new Solenoid(4);
		m_climbR = new Solenoid(5);

		m_PDServo1 = new Servo(9);
		m_PDServo2 = new Servo(7);

		m_Joystick = new Joystick(0);
		m_Gamepad = new Joystick(1);
		m_Gamepad2 = new Joystick(2);

		shooter1 = new CANTalon(0);
		shooter1->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		shooter1->SetControlMode(CANTalon::kSpeed);
		shooter1->SetCloseLoopRampRate(0);
		shooter1->ConfigEncoderCodesPerRev(4096);
		shooter1->SetPID(0.08, 0, 2, 0);
		shooter1->SelectProfileSlot(0);
		// shooter1->SetSensorDirection(true);
		shooter1->SetAllowableClosedLoopErr(1000);

		shooter2 = new CANTalon(1);
		shooter2->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		shooter2->SetControlMode(CANTalon::kSpeed);
		shooter2->SetCloseLoopRampRate(0);
		shooter2->ConfigEncoderCodesPerRev(4096);
		shooter2->SetPID(0.08, 0, 2, 0);
		shooter2->SelectProfileSlot(0);
		// shooter2->SetSensorDirection(false);
		shooter2->SetAllowableClosedLoopErr(1000);

		m_intakeRoller = new VictorSP(8);

		m_LED = new Relay(0);

		drivePID = new SimPID(0.0002 ,0, 0.00001 , 500);
		drivePID->setMinDoneCycles(10);
		drivePID->setMaxOutput(0.75);

		turnPID = new SimPID(0.075, 0, 0, 5);
		turnPID->setMinDoneCycles(1);
		turnPID->setMaxOutput(0.5);
		turnPID->setContinuousAngle(true);

		turnPID2 = new SimPID(0.058 ,0.06,0.0,0.75);
		turnPID2->setMinDoneCycles(10);
		turnPID2->setMaxOutput(0.5);
		turnPID2->setMinOutput(0.12);
		turnPID2->setContinuousAngle(true);

		shooterPID = new SimPID(0.0025, 0, 0.0001,10);
		shooterPID->setMaxOutput(0.36);


		intakePID = new SimPID(0.001, 0, 0.001, 10);
		intakePID->setMaxOutput(0.7);

		visionPID = new SimPID(0.01, 0.02, 0.001, 5);
		visionPID->setMaxOutput(0.5);
		visionPID->setMinDoneCycles(20);

		m_shooterHomeSwitch = new DigitalInput(0);
		m_intakeHomeSwitch = new DigitalInput(1);

		m_leftDriveEncoder = new Encoder(2, 3);
		m_rightDriveEncoder = new Encoder(5, 4);

		m_shooter = new CANTalon(2);
		m_shooter->SetFeedbackDevice(CANTalon::QuadEncoder);
		//m_shooter->ConfigEncoderCodesPerRev(4096);
		m_shooter->SetSensorDirection(true);

		m_intake = new CANTalon(4);
		//m_intake->SetFeedbackDevice(CANTalon::QuadEncoder);
		m_intake->ConfigEncoderCodesPerRev(4096);

		nav = new AHRS(SPI::Port::kMXP);

		m_climber = new VictorSP(5);

		PDP = new PowerDistributionPanel(0);

		timer = new Timer();
		timer->Reset();
		stateTimer = new Timer();
		stateTimer->Reset();
		trimTimer = new Timer();
		trimTimer->Reset();
		pressTimer = new Timer();
		pressTimer->Reset();

		VisionInit();


		shooterState = 0;
		autoState = 0;
		autoMode = 0;
		autoPosition = 0;
		pusherState = 0;
		aim_fly_trim = 0;
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
		filterOptions.fillHoles = FALSE;
		filterOptions.rejectBorder = FALSE;
		filterOptions.rejectMatches = FALSE;

		//common filter config
		for (int i = 0; i < CRITERIA_COUNT; i++){
			filterCriteria[i].calibrated = FALSE;
			filterCriteria[i].exclude = FALSE;
		}

		//area config
		filterCriteria[0].parameter = IMAQ_MT_AREA;
		filterCriteria[0].lower = 1500;
		filterCriteria[0].upper = 1900;

		//width config
		filterCriteria[1].parameter = IMAQ_MT_BOUNDING_RECT_WIDTH;
		filterCriteria[1].lower = 90;
		filterCriteria[1].upper = 130;

		//height config
		filterCriteria[2].parameter = IMAQ_MT_BOUNDING_RECT_HEIGHT;
		filterCriteria[2].lower = 40;
		filterCriteria[2].upper = 80;

		//add perimeter filter
		filterCriteria[3].parameter = IMAQ_MT_PERIMETER;
		filterCriteria[3].lower = 30;
		filterCriteria[3].upper = 450;
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
		B = raw_pixel;
		G = raw_pixel + 1;
		R = raw_pixel + 2;
		alpha = raw_pixel + 3;

		//misc set up
		colourTable = {255,255,255,0};
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
		stateTimer->Stop();
		m_LED->Set(Relay::kOff);
	}

	void DisabledPeriodic()
	{
		//FindTargetCenter();
		//printf("shooter Speed: %f\t%f\n", shooter1->GetSpeed(), shooter2->GetSpeed());

		//if(m_shooterHomeSwitch->Get() == CLOSED)
			//m_shooter->SetPosition(0);

		printf("\n r %f", nav->GetRoll());
		printf("\n p %f", nav->GetPitch());



		if(m_Joystick->GetRawButton(10)){
			printf("RESETTING ENCODERS\n");
			m_shooter->SetPosition(0);
			//m_shooter->SetEncPosition(0);
			m_intake->SetPosition(0);
			//m_intake->SetEncPosition(0);
			shooter1->SetPosition(0);
			shooter2->SetPosition(0);
		}
		if(m_Joystick->GetPOV() == JS_LEFT)
		{
			autoDirection = 0;
			DriverStation::ReportError("Auto mode: " + std::to_string((long)autoMode) + " position: " + std::to_string((long)autoPosition) + "\n" + "Direction: " + std::to_string((long)autoDirection) +  "\n" + "autoDelay" + std::to_string((float)autoDelay));
		}
		else if(m_Joystick->GetPOV() == JS_RIGHT)
		{
			DriverStation::ReportError("Auto mode: " + std::to_string((long)autoMode) + " position: " + std::to_string((long)autoPosition) + "\n" + "Direction: " + std::to_string((long)autoDirection) +  "\n" + "autoDelay" + std::to_string((float)autoDelay));
			autoDirection = 1;
		}

		for (int i = 1; i <= 12; i++)
		{
			if(m_Joystick->GetRawButton(i))
			{
				if (i < 9)
					autoMode = i;
				else if (i >= 9 && i <= 12)
					autoPosition = i - 7;

				nav->Reset();
				autoDelay = -5*(m_Joystick->GetRawAxis(3)) + 5;
				m_leftDriveEncoder->Reset();
				m_rightDriveEncoder->Reset();
				DriverStation::ReportError("Auto mode: " + std::to_string((long)autoMode) + " position: " + std::to_string((long)autoPosition) + "\n" + "Direction: " + std::to_string((long)autoDirection) +  "\n" + "autoDelay" + std::to_string((float)autoDelay));
			}
		}


		DriverStation::ReportError("Gyro: " + std::to_string((float)nav->GetYaw()) +
				" enc: " + std::to_string((long)m_leftDriveEncoder->Get()) +
				", " + std::to_string((long)m_rightDriveEncoder->Get()) + "\n");



		//printf("shooterA: %d\tintakeA: %d\n", m_shooter->GetEncPosition(), m_intake->GetEncPosition());
		//printf("shooterA: %f\tintakeA: %f\n", m_shooter->GetPosition(), m_intake->GetPosition());

		if(m_Gamepad->GetRawButton(GP_R))
			autoPosition = 6;
	}

	//========================================================AUTONOMOUS=======================================
	void AutonomousInit(void)
	{
		autoState = 0;
		nav->Reset();
		m_leftDriveEncoder->Reset();
		m_rightDriveEncoder->Reset();
		shooter1->Set(0.f);
		shooter2->Set(0.f);
		timer->Reset();
		timer->Start();
		stateTimer->Reset();
		stateTimer->Start();
		IMAQdxStartAcquisition(session);
		m_LED->Set(Relay::kForward);
	}

	void AutonomousPeriodic(void)
	{
		printf("auto state: %d", autoState);
		printf("\tleft: %d\t right: %d\t angle: %f\n", m_leftDriveEncoder->Get(), m_rightDriveEncoder->Get(), nav->GetYaw());
		printf("\t\tdrive done %d\tturn done: %d\n", drivePID->isDone(), turnPID->isDone());


		if(stateTimer->Get() > autoDelay)
		{
			switch(autoMode)
			{
			case 1:
				m_leftDrive4->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);

				autoShooter(HOME_SHOOTER);
				autoIntake(HOME_INTAKE);

				m_intakeRoller->SetSpeed(0.f);

				break;
			case 0: //drive forward with boulder preloaded and move under low bar
				switch(autoState)
				{
				case 0:
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(HOME_INTAKE);
					m_intakeRoller->SetSpeed(0.f);
					timer->Reset();
					timer->Start();
					autoState++;
					break;
				case 2: //intake to pickup
					autoShooter(HOME_SHOOTER);
					m_shootE->Set(true);
					m_shootR->Set(false);

					autoIntake(PICKUP);

					if(timer->Get() > 1.0)
						autoState++;
					break;
				case 1: //drive under lowbar
					autoShooter(HOME_SHOOTER);
					m_shootE->Set(true);
					m_shootR->Set(false);

					autoIntake(PICKUP);
					autoDrive(AUTO_LOWBAR_DRIVE, 0);
					break;
				}
				break;

			case 9: //drive forward with boulder preloaded, cross low bar, discharge ball to tower, revere to neutral zone through low bar, ready to get next boulder
				printf("enc: %d\t gyro: %f",(m_rightDriveEncoder->Get() + m_leftDriveEncoder->Get())/2, nav->GetYaw());
				switch(autoState)
				{
				case 0:
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(HOME_INTAKE);
					m_intakeRoller->SetSpeed(0.f);

					timer->Reset();
					timer->Start();

					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);

					autoState++;
					break;
				case 1: //intake down
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.75)
						{
							autoState++;
							timer->Reset();
							timer->Stop();
						}
					break;
				case 2: //drive under lowbar
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);
					if(autoDrive(18000, 0))
						autoState++;
					break;
				case 3: //turn towards castle
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					if(autoDrive(19000, 55))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 4: //drive towards castle
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					if(autoDrive(23000, 55))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 5: //discharge ball
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(ROLLER_SPEED);

					if(timer->Get() > 2.0)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 6: //drive back
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					if(autoDrive(19000, 55))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 7: //turn around
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					if(autoDrive(19000, 180))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;

				case 8: //return under lowbar
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					autoDrive(38000, 180);
					break;

				case 9: //turn towards boulders
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					autoDrive(38000, 135);
					break;

				}
				break;
			case 3: // drive over flat defense in any position and shoot high goal/halt
				printf("autoState: %d\n", autoState);
				switch(autoState){
				case 0:
					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					autoState++;
					break;
				case 1: //drive over defense
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_OTHER, 0) && nav->GetRoll() > -10 && nav->GetRoll() < 10){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 2: //aim at tower roughly and prep ball 1
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					int result;
					switch(autoPosition){
					case 2:

						switch(autoDirection){
						case 0:
							autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_2_L);
							break;
						case 1:
							autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_2_R);
							break;

						}

						break;
					case 3:
						result = autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_3);
						break;
					case 4:
						result = autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_4);
						break;
					case 5:
						result = autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_5);

						break;
					case 6:
						autoState = 5;
						break;
					}
					if(autoPosition == 2 && autoDirection == 0 && timer->Get() > 1.0)
						{
							timer->Reset();
							timer->Start();
							autoState = 6;
						}
					else if(autoPosition == 2 && autoDirection == 1 && timer->Get() > 1.0)
						{
							timer->Reset();
							timer->Start();
							autoState = 9;
						}
					else if(result && timer->Get() > 1.0)
					{
						autoState = 11;
						timer->Reset();
						timer->Start();
					}
					break;
				case 3: //prep ball 2, confirm aim with vision
					FindTargetCenter();
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					shooterState = 60;
					if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState = 4;
						timer->Reset();
						timer->Start();
					}
					else if(timer->Get() > 2 && abs(centerx-(RES_X/2)+AIM_CORRECTION) < 3){
						timer->Reset();
						timer->Start();
						autoState = 13;
					}


					break;
				case 4: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//m_shooterServo->SetAngle(SERVO_OUT);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.3)
					{
						autoState = 5;
						timer->Start();
						timer->Reset();
					}
					break;
				case 5: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.5)
						autoState = 14;
					break;
				case 6: //position 2_L extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_OTHER - 2000, AUTO_AIM_POS_2_L))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}


					break;
				case 7: //position 2_L rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_OTHER - 2000, 35) && timer->Get() > 1.0)
						{
							autoAimAngle = getAutoAimAngle();
							timer->Reset();
							timer->Start();
							autoState = 3;
						}
					break;
				/*case 8: //position 2_R extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_MOAT - 2500, AUTO_AIM_POS_2_R))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}


					break;*/
				case 9: //position 2_R rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_2_R) && timer->Get() > 1.0)
						{
							autoAimAngle = getAutoAimAngle();
							timer->Reset();
							timer->Start();
							autoState = 3;
						}
					break;

				case 11: //wait a bit before taking picture
					autoShooter(SHOOT_FAR);
					if(timer->Get() > 1.0){
						autoAimAngle = getAutoAimAngle();

						/*if(autoPosition == 2)
						{
							timer->Reset();
							timer->Start();
							autoState = 6;
						}
						if(autoPosition == 5)
						{
							timer->Reset();
							timer->Start();
							autoState = 8;
						}*/
						timer->Reset();
						timer->Start();
						autoState = 3;

					}
					break;

				case 13: //wait before taking shot
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(timer->Get() > 0.75){
						timer->Reset();
						timer->Start();
						autoState = 4;
					}

					break;
				case 14: //drive to neutral zone
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(NEUTRAL_DRIVE, 0))
						autoState++;
					break;
				case 15: //lower intake
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoDrive(NEUTRAL_DRIVE, 0);
					break;
				}
				break;

			case 4: // drive over moat defense in any position and shoot high goal/halt
				printf("autoState: %d\n", autoState);
				switch(autoState){
				case 0:
					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					autoState++;
					break;
				case 1: //drive over defense
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_MOAT, 0) && nav->GetRoll() > -10 && nav->GetRoll() < 10){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 2: //aim at tower roughly and prep ball 1
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					int result;
					switch(autoPosition){
					case 2:

						switch(autoDirection){
						case 0:
							autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_2_L);
							break;
						case 1:
							autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_2_R);
							break;

						}

						break;
					case 3:
						result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_3);
						break;
					case 4:
						result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_4);
						break;
					case 5:
						result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_5);

						break;
					case 6:
						autoState = 5;
						break;
					}
					if(autoPosition == 2 && autoDirection == 0 && timer->Get() > 1.0)
						{
							timer->Reset();
							timer->Start();
							autoState = 6;
						}
					else if(autoPosition == 2 && autoDirection == 1 && timer->Get() > 1.0)
						{
							timer->Reset();
							timer->Start();
							autoState = 9;
						}
					else if(result && timer->Get() > 1.0)
					{
						autoState = 11;
						timer->Reset();
						timer->Start();
					}
					break;
				case 3: //prep ball 2, confirm aim with vision
					FindTargetCenter();
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					shooterState = 60;
					if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState = 4;
						timer->Reset();
						timer->Start();
					}
					else if(timer->Get() > 2 && abs(centerx-(RES_X/2)+AIM_CORRECTION) < 3){
						timer->Reset();
						timer->Start();
						autoState = 13;
					}


					break;
				case 4: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//m_shooterServo->SetAngle(SERVO_OUT);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.3)
					{
						autoState = 5;
						timer->Start();
						timer->Reset();
					}
					break;
				case 5: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.5)
						autoState = 14;
					break;
				case 6: //position 2_L extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_MOAT - 2000, AUTO_AIM_POS_2_L))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}


					break;
				case 7: //position 2_L rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_MOAT - 2000, 35) && timer->Get() > 1.0)
						{
							autoAimAngle = getAutoAimAngle();
							timer->Reset();
							timer->Start();
							autoState = 3;
						}
					break;
				/*case 8: //position 2_R extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_MOAT - 2500, AUTO_AIM_POS_2_R))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}


					break;*/
				case 9: //position 2_R rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_2_R) && timer->Get() > 1.0)
						{
							autoAimAngle = getAutoAimAngle();
							timer->Reset();
							timer->Start();
							autoState = 3;
						}
					break;

				case 11: //wait a bit before taking picture
					autoShooter(SHOOT_FAR);
					if(timer->Get() > 1.0){
						autoAimAngle = getAutoAimAngle();

						/*if(autoPosition == 2)
						{
							timer->Reset();
							timer->Start();
							autoState = 6;
						}
						if(autoPosition == 5)
						{
							timer->Reset();
							timer->Start();
							autoState = 8;
						}*/
						timer->Reset();
						timer->Start();
						autoState = 3;

					}
					break;

				case 13: //wait before taking shot
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(timer->Get() > 0.75){
						timer->Reset();
						timer->Start();
						autoState = 4;
					}

					break;
				case 14: //drive to neutral zone
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(NEUTRAL_DRIVE, 0))
						autoState++;
					break;
				case 15: //lower intake
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoDrive(NEUTRAL_DRIVE, 0);
					break;
				}
				break;

			case 5: // drive over rough defense in any position and shoot high goal/halt
				printf("autoState: %d\n", autoState);
				switch(autoState){
				case 0:
					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					autoState++;
					break;
				case 1: //drive over defense
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_ROUGH, 0) && nav->GetRoll() > -10 && nav->GetRoll() < 10){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 2: //aim at tower roughly and prep ball 1
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					int result;
					switch(autoPosition){
					case 2:

						switch(autoDirection){
						case 0:
							autoDrive(AUTO_OVER_ROUGH, AUTO_AIM_POS_2_L);
							break;
						case 1:
							autoDrive(AUTO_OVER_ROUGH, AUTO_AIM_POS_2_R);
							break;

						}

						break;
					case 3:
						result = autoDrive(AUTO_OVER_ROUGH, AUTO_AIM_POS_3);
						break;
					case 4:
						result = autoDrive(AUTO_OVER_ROUGH, AUTO_AIM_POS_4);
						break;
					case 5:
						result = autoDrive(AUTO_OVER_ROUGH, AUTO_AIM_POS_5);

						break;
					case 6:
						autoState = 5;
						break;
					}
					if(autoPosition == 2 && autoDirection == 0 && timer->Get() > 1.0)
						{
							timer->Reset();
							timer->Start();
							autoState = 6;
						}
					else if(autoPosition == 2 && autoDirection == 1 && timer->Get() > 1.0)
						{
							timer->Reset();
							timer->Start();
							autoState = 9;
						}
					else if(result && timer->Get() > 1.0)
					{
						autoState = 11;
						timer->Reset();
						timer->Start();
					}
					break;
				case 3: //prep ball 2, confirm aim with vision
					FindTargetCenter();
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					shooterState = 60;
					if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState = 4;
						timer->Reset();
						timer->Start();
					}
					else if(timer->Get() > 2 && abs(centerx-(RES_X/2)+AIM_CORRECTION) < 3){
						timer->Reset();
						timer->Start();
						autoState = 13;
					}


					break;
				case 4: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//m_shooterServo->SetAngle(SERVO_OUT);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.3)
					{
						autoState = 5;
						timer->Start();
						timer->Reset();
					}
					break;
				case 5: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.5)
						autoState = 14;
					break;
				case 6: //position 2_L extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_ROUGH - 2000, AUTO_AIM_POS_2_L))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}


					break;
				case 7: //position 2_L rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_ROUGH - 2000, 35) && timer->Get() > 1.0)
						{
							autoAimAngle = getAutoAimAngle();
							timer->Reset();
							timer->Start();
							autoState = 3;
						}
					break;
				/*case 8: //position 2_R extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_MOAT - 2500, AUTO_AIM_POS_2_R))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}


					break;*/
				case 9: //position 2_R rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_ROUGH, AUTO_AIM_POS_2_R) && timer->Get() > 1.0)
						{
							autoAimAngle = getAutoAimAngle();
							timer->Reset();
							timer->Start();
							autoState = 3;
						}
					break;

				case 11: //wait a bit before taking picture
					autoShooter(SHOOT_FAR);
					if(timer->Get() > 1.0){
						autoAimAngle = getAutoAimAngle();

						/*if(autoPosition == 2)
						{
							timer->Reset();
							timer->Start();
							autoState = 6;
						}
						if(autoPosition == 5)
						{
							timer->Reset();
							timer->Start();
							autoState = 8;
						}*/
						timer->Reset();
						timer->Start();
						autoState = 3;

					}
					break;

				case 13: //wait before taking shot
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(timer->Get() > 0.75){
						timer->Reset();
						timer->Start();
						autoState = 4;
					}

					break;
				case 14: //drive to neutral zone
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(NEUTRAL_DRIVE, 0))
						autoState++;
					break;
				case 15: //lower intake
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoDrive(NEUTRAL_DRIVE, 0);
					break;
				}
				break;

			case 6: //under lowbar, shoot with vision high goal
				printf("autoState: %d\n", autoState);
				switch(autoState)
				{
				case 0:
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					timer->Reset();
					timer->Start();
					autoState++;
					break;
				case 1: //intake down
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 0.75)
						{
							autoState++;
						}
					break;
				case 2: //drive under lowbar
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
						{
							timer->Reset();
							timer->Start();
							autoState++;

						}
					break;
				case 3: //move intake to transfer
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 0.5)
					{
						timer->Reset();
						timer->Start();
						autoState++;
					}

					break;
				case 4: //transfer
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(SPEED_RPM/6);
					shooter2->Set(-SPEED_RPM/6);
					m_intakeRoller->SetSpeed(-ROLLER_SPEED);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 0.5)
					{
						timer->Reset();
						timer->Start();
						autoState++;
					}
					break;
				case 5: //rough turn
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, AUTO_AIM_POS_1) && timer->Get() > 1.5)
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}
					break;
				case 6: //prep ball 1
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoAimAngle = getAutoAimAngle();
					//m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 1.0)
					{
						timer->Reset();
						timer->Start();
						autoState++;
					}

					break;
				case 7: //prep ball 2
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(-SPEED_RPM/6);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					timer->Start();
					timer->Reset();
					//m_shooterServo->SetAngle(SERVO_IN)
					autoState++;
					break;
				case 8: //confirm aim with vision
					FindTargetCenter();
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_intakeRoller->SetSpeed(0.f);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					shooterState = 60;
					if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					else if(timer->Get() > 2 && abs(centerx-(RES_X/2)+AIM_CORRECTION) < 3){
						timer->Reset();
						timer->Start();
						autoState = 11;
					}
					break;
				case 9: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);

					if(timer->Get() > 0.5)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 10: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoState = 12;
					//m_shooterServo->SetAngle(SERVO_IN);
					break;
				case 11: //wait before taking shot
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_leftDrive4->SetSpeed(0.f);
					m_leftDrive1->SetSpeed(0.f);
					m_rightDrive2->SetSpeed(0.f);
					m_rightDrive3->SetSpeed(0.f);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(timer->Get() > 0.5){
						timer->Reset();
						timer->Start();
						autoState = 9;
					}

					break;
				case 12: //drive back to neutral zone
					autoIntake(PICKUP);
					autoShooter(HOME_SHOOTER);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(NEUTRAL_DRIVE, 0))
						autoState++;
					break;
				case 13: //lower intake
					autoIntake(PICKUP);
					autoShooter(HOME_SHOOTER);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoDrive(-5000, 0);
					break;
				}
				break;

				case 7: // drive over chevel in any position and shoot high goal/halt
					printf("autoState: %d\n", autoState);
					switch(autoState){
					case 0:
						drivePID->setMaxOutput(0.3);
						m_leftDrive4->SetSpeed(0.f);
						m_leftDrive1->SetSpeed(0.f);
						m_rightDrive2->SetSpeed(0.f);
						m_rightDrive3->SetSpeed(0.f);
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						autoState++;
						break;
					case 1: //drive towards cheval
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_1, 0) && nav->GetRoll() > -10 && nav->GetRoll() < 10){
							autoState++;
							timer->Reset();
							timer->Start();
						}
						break;
					case 2: //lower intake
						autoShooter(HOME_SHOOTER);
						autoIntake(PICKUP);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if(timer->Get() > 0.5)
						{
							timer->Reset();
							timer->Start();
							drivePID->setMaxOutput(0.7);
							autoState++;
						}
						break;
					case 3: //drive to courtyard
						autoShooter(HOME_SHOOTER);

						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if((m_leftDriveEncoder->Get() + m_rightDriveEncoder->Get())/2 >  AUTO_CHEVAL_DRIVE_1 + 1200){
							autoIntake(HOME_INTAKE);
						}
						else
							autoIntake(PICKUP);

						if(autoDrive(AUTO_CHEVAL_DRIVE_2, 0))
						{
							turnPID->setMaxOutput(1.0);
							timer->Reset();
							timer->Start();
							autoState++;
						}

						if((nav->GetPitch() < 13 && timer->Get() > 1.5)){
							m_leftDrive4->SetSpeed(0.f);
							m_leftDrive1->SetSpeed(0.f);
							m_rightDrive2->SetSpeed(0.f);
							m_rightDrive3->SetSpeed(0.f);
						}
						break;
					case 4: //spin around
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if(autoDrive(AUTO_CHEVAL_DRIVE_2, 180))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}
						break;
					case 5: //aim at tower roughly and prep ball 1
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						int result;
						switch(autoPosition){
						case 2:

							switch(autoDirection){
							case 0:
								autoDrive(AUTO_CHEVAL_DRIVE_2, AUTO_AIM_POS_2_L + 180);
								break;
							case 1:
								autoDrive(AUTO_CHEVAL_DRIVE_2, AUTO_AIM_POS_2_R + 180);
								break;

							}

							break;
						case 3:
							result = autoDrive(AUTO_CHEVAL_DRIVE_2, AUTO_AIM_POS_3 + 180);
							break;
						case 4:
							result = autoDrive(AUTO_CHEVAL_DRIVE_2, AUTO_AIM_POS_4 + 180);
							break;
						case 5:
							result = autoDrive(AUTO_CHEVAL_DRIVE_2, AUTO_AIM_POS_5 + 180);

							break;
						case 6:
							autoState = 8;
							break;
						}
						if(autoPosition == 2 && autoDirection == 0 && timer->Get() > 1.5)
							{
								timer->Reset();
								timer->Start();
								autoState = 9;
							}
						else if(autoPosition == 2 && autoDirection == 1 && timer->Get() > 1.5)
							{
								timer->Reset();
								timer->Start();
								autoState = 11;
							}
						else if(result && timer->Get() > 1.0)
						{
							autoState = 13;
							timer->Reset();
							timer->Start();
						}
						break;
					case 6: //prep ball 2, confirm aim with vision
						FindTargetCenter();
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM - BALL_SPIN);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
							autoState++;
							timer->Reset();
							timer->Start();
						}
						else if(timer->Get() > 2 && abs(centerx-(RES_X/2)+AIM_CORRECTION) < 3){
							timer->Reset();
							timer->Start();
							autoState = 15;
						}

						shooterState = 60;
						break;
					case 7: //shoot the ball
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM - BALL_SPIN);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(true);
						m_shootR->Set(false);
						//m_shooterServo->SetAngle(SERVO_OUT);
						m_intakeRoller->SetSpeed(0.f);
						if(timer->Get() > 0.3)
						{
							autoState++;
							timer->Start();
							timer->Reset();
						}
						break;
					case 8: //turn off shooter
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						m_intakeRoller->SetSpeed(0.f);

						break;
					case 9: //position 2_L extra drive
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_2 - 2000, AUTO_AIM_POS_2_L))
							{
								timer->Reset();
								timer->Start();
								autoState++;
							}


						break;
					case 10: //position 2_L rough angle
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_2 - 2000, 35) && timer->Get() > 1.0)
							{
								autoAimAngle = getAutoAimAngle();
								timer->Reset();
								timer->Start();
								autoState = 6;
							}
						break;
					/*case 8: //position 2_R extra drive
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_2 - 2500, AUTO_AIM_POS_2_R))
							{
								timer->Reset();
								timer->Start();
								autoState++;
							}


						break;*/
					case 11 : //position 2_R rough angle
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_2, AUTO_AIM_POS_2_R) && timer->Get() > 1.0)
							{
								autoAimAngle = getAutoAimAngle();
								timer->Reset();
								timer->Start();
								autoState = 6;
							}
						break;

/*					case 12: //back up before shot
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_2, 20) && timer->Get() > 0.5)
							{
								autoAimAngle = getAutoAimAngle();
								timer->Reset();
								timer->Start();
								autoState = 3;
							}
						break;
*/
					case 13: //wait a bit before taking picture
						autoShooter(SHOOT_FAR);
							if(timer->Get() > 1.0){
							autoAimAngle = getAutoAimAngle();

							timer->Reset();
							timer->Start();
							autoState = 6;

						}
						break;

					case 14: //go to home
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						m_intakeRoller->SetSpeed(0.f);
						shooterState = 10;
						break;
					case 15: //wait before taking shot
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM - BALL_SPIN);
						m_leftDrive4->SetSpeed(0.f);
						m_leftDrive1->SetSpeed(0.f);
						m_rightDrive2->SetSpeed(0.f);
						m_rightDrive3->SetSpeed(0.f);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if(timer->Get() > 0.75){
							timer->Reset();
							timer->Start();
							autoState = 7;
						}

						break;
					}
					break;
				case 8: // drive over portcullis in any position and shoot high goal/halt
					printf("autoState: %d\n", autoState);
					switch(autoState){
					case 0:
						drivePID->setMaxOutput(0.4);
						m_leftDrive4->SetSpeed(0.f);
						m_leftDrive1->SetSpeed(0.f);
						m_rightDrive2->SetSpeed(0.f);
						m_rightDrive3->SetSpeed(0.f);
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						autoState++;
						break;
					case 1: //drive towards portcullis
						autoShooter(HOME_SHOOTER);
						autoIntake(PICKUP);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_PORTCULLIS_DRIVE_1, 0) && nav->GetRoll() > -10 && nav->GetRoll() < 10){
							autoState++;
							timer->Reset();
							timer->Start();
						}
						break;
					case 2: //raise intake
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if(timer->Get() > 0.5)
						{
							timer->Reset();
							timer->Start();
							drivePID->setMaxOutput(0.7);
							autoState++;
						}
						break;
					case 3: //drive to courtyard
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);

						if(autoDrive(AUTO_PORTCULLIS_DRIVE_2, 0))
						{
							turnPID->setMaxOutput(1.0);
							timer->Reset();
							timer->Start();
							autoState++;
						}

						if((m_leftDriveEncoder->Get() + m_rightDriveEncoder->Get())/2 < AUTO_PORTCULLIS_DRIVE_1 + 500 && timer->Get() > 3){
							m_leftDrive4->SetSpeed(0.f);
							m_leftDrive1->SetSpeed(0.f);
							m_rightDrive2->SetSpeed(0.f);
							m_rightDrive3->SetSpeed(0.f);
						}

						break;

					case 4: //spin around
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if(autoDrive(AUTO_PORTCULLIS_DRIVE_2, 180))
						{
							timer->Reset();
							timer->Start();
							autoState++;
						}
						break;
					case 5: //aim at tower roughly and prep ball 1
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						int result;
						switch(autoPosition){
						case 2:

							switch(autoDirection){
							case 0:
								autoDrive(AUTO_PORTCULLIS_DRIVE_2, AUTO_AIM_POS_2_L + 180);
								break;
							case 1:
								autoDrive(AUTO_PORTCULLIS_DRIVE_2, AUTO_AIM_POS_2_R + 180);
								break;

							}

							break;
						case 3:
							result = autoDrive(AUTO_PORTCULLIS_DRIVE_2, AUTO_AIM_POS_3 + 180);
							break;
						case 4:
							result = autoDrive(AUTO_PORTCULLIS_DRIVE_2, AUTO_AIM_POS_4 + 180);
							break;
						case 5:
							result = autoDrive(AUTO_PORTCULLIS_DRIVE_2, AUTO_AIM_POS_5 + 180);

							break;
						case 6:
							autoState = 8;
							break;
						}
						if(autoPosition == 2 && autoDirection == 0 && timer->Get() > 1.5)
							{
								timer->Reset();
								timer->Start();
								autoState = 9;
							}
						else if(autoPosition == 2 && autoDirection == 1 && timer->Get() > 1.5)
							{
								timer->Reset();
								timer->Start();
								autoState = 11;
							}
						else if(result && timer->Get() > 1.0)
						{
							autoState = 13;
							timer->Reset();
							timer->Start();
						}
						break;
					case 6: //prep ball 2, confirm aim with vision
						FindTargetCenter();
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM - BALL_SPIN);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
							autoState++;
							timer->Reset();
							timer->Start();
						}
						else if(timer->Get() > 2 && abs(centerx-(RES_X/2)+AIM_CORRECTION) < 3){
							timer->Reset();
							timer->Start();
							autoState = 15;
						}

						shooterState = 60;
						break;
					case 7: //shoot the ball
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM - BALL_SPIN);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(true);
						m_shootR->Set(false);
						//m_shooterServo->SetAngle(SERVO_OUT);
						m_intakeRoller->SetSpeed(0.f);
						if(timer->Get() > 0.3)
						{
							autoState++;
							timer->Start();
							timer->Reset();
						}
						break;
					case 8: //turn off shooter
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						m_intakeRoller->SetSpeed(0.f);

						break;
					case 9: //position 2_L extra drive
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_PORTCULLIS_DRIVE_2 - 2000, AUTO_AIM_POS_2_L))
							{
								timer->Reset();
								timer->Start();
								autoState++;
							}


						break;
					case 10: //position 2_L rough angle
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_PORTCULLIS_DRIVE_2 - 2000, 35) && timer->Get() > 1.0)
							{
								autoAimAngle = getAutoAimAngle();
								timer->Reset();
								timer->Start();
								autoState = 6;
							}
						break;
					/*case 8: //position 2_R extra drive
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_2 - 2500, AUTO_AIM_POS_2_R))
							{
								timer->Reset();
								timer->Start();
								autoState++;
							}


						break;*/
					case 11 : //position 2_R rough angle
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_PORTCULLIS_DRIVE_2, AUTO_AIM_POS_2_R) && timer->Get() > 1.0)
							{
								autoAimAngle = getAutoAimAngle();
								timer->Reset();
								timer->Start();
								autoState = 6;
							}
						break;

/*					case 12: //back up before shot
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(AUTO_CHEVAL_DRIVE_2, 20) && timer->Get() > 0.5)
							{
								autoAimAngle = getAutoAimAngle();
								timer->Reset();
								timer->Start();
								autoState = 3;
							}
						break;
*/
					case 13: //wait a bit before taking picture
						autoShooter(SHOOT_FAR);
							if(timer->Get() > 1.0){
							autoAimAngle = getAutoAimAngle();

							timer->Reset();
							timer->Start();
							autoState = 6;

						}
						break;

					case 14: //go to home
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						m_intakeRoller->SetSpeed(0.f);
						shooterState = 10;
						break;
					case 15: //wait before taking shot
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM - BALL_SPIN);
						m_leftDrive4->SetSpeed(0.f);
						m_leftDrive1->SetSpeed(0.f);
						m_rightDrive2->SetSpeed(0.f);
						m_rightDrive3->SetSpeed(0.f);
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if(timer->Get() > 0.75){
							timer->Reset();
							timer->Start();
							autoState = 7;
						}

						break;
					}
					break;

			case 2: //2-ball auto
				printf("autoState: %d\n", autoState);
				switch(autoState)
				{
				case 0:
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					timer->Reset();
					timer->Start();
					autoState++;
					break;
				case 1: //intake down and move intake rollers
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_intakeRoller->Set(-ROLLER_SPEED);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 3.0)
					{
						autoState++;
						timer->Reset();
						timer->Stop();
					}
					break;
				case 2: //drive under lowbar
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 3: //move intake to transfer
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 0.5)
						autoState++;
					break;
				case 4: //rough turn, transfer ball
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(-ROLLER_SPEED);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, AUTO_AIM_POS_1))
						autoState++;
					break;
				case 5: //prep ball 1
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoAimAngle = getAutoAimAngle();
					//m_shooterServo->SetAngle(SERVO_IN);
						autoState++;
					break;
				case 6: //prep ball 2
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN)
					autoState++;
					break;
				case 7: //confirm aim with vision
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_intakeRoller->SetSpeed(0.f);
					autoShooter(findShooterAngle());
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 8: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(findShooterAngle());
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
				//	m_shooterServo->SetAngle(SERVO_OUT);

					if(timer->Get() > 0.5)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 9: //turn off shooter, turn straight again
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
						autoState++;
					break;
				case 10: //drive back under lowbar, move intake rollers to suck ball in
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					m_intakeRoller->Set(-ROLLER_SPEED);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(0, 0))
						autoState++;
					break;
				case 11: //drive under lowbar
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 12: //move intake to transfer
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 1.5)
						autoState++;
					break;
				case 13: //rough turn, transfer ball
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(-ROLLER_SPEED);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, AUTO_AIM_POS_1))
						autoState++;
					break;
				case 14: //prep ball 1
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					autoAimAngle = getAutoAimAngle();
					//m_shooterServo->SetAngle(SERVO_IN);
						autoState++;
					break;
				case 15: //prep ball 2
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN)
					autoState++;
					break;
				case 16: //confirm aim with vision
					FindTargetCenter();
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					m_intakeRoller->SetSpeed(0.f);
					autoShooter(findShooterAngle());
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 17: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM - BALL_SPIN);
					autoShooter(findShooterAngle());
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
				//	m_shooterServo->SetAngle(SERVO_OUT);

					if(timer->Get() > 0.5)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 18: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					m_shootE->Set(false);
					m_shootR->Set(true);
					break;

				}
				break;
			case 10: //alt 2-ball
				switch(autoState)
				{
				case 0:
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					autoState++;
					break;
				case 1: //move away from ball
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					if(autoDrive(10000, 0))
						autoState++;
					break;
				case 2: //lower intake, move under lowbar
					autoIntake(PICKUP);
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
						autoState++;
					break;
				case 3: //prep ball
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(AUTO_LOWBAR_DRIVE, AUTO_LOWBAR_ANGLE))
					{
						timer->Reset();
						timer->Start();
						autoState++;
					}
					break;
				case 4: //run shooters
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(SHOOT_SPEED);
					shooter2->Set(SHOOT_SPEED);
					m_intakeRoller->SetSpeed(ROLLER_SPEED);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(timer->Get() > 1)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 5: //shoot
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(SHOOT_SPEED);
					shooter2->Set(SHOOT_SPEED);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					if(timer->Get() > 0.5)
						autoState++;
					break;
				case 6: //reset turn and lower intake
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
						autoState++;
					break;
				case 7: //drive back, pick up ball
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(ROLLER_SPEED);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(0, 0))
						autoState++;
					break;
				case 8: //lower intake, move under lowbar
					autoIntake(PICKUP);
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
						autoState++;
					break;
				case 9: //home intake
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoIntake(HOME_INTAKE))
					{
						timer->Start();
						autoState++;
					}
					break;
				case 10: //prep ball 1
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(ROLLER_SPEED);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(timer->Get() > 0.5)
					{
						timer->Reset();
						timer->Stop();
						autoState++;
					}
					break;
				case 11: //prep ball 2
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(AUTO_LOWBAR_DRIVE, AUTO_LOWBAR_ANGLE))
					{
						timer->Reset();
						timer->Start();
						autoState++;
					}
					break;
				case 12: //run shooters
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(SHOOT_SPEED);
					shooter2->Set(SHOOT_SPEED);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(timer->Get() > 1)
					{
						autoState++;
						timer->Reset();
					}
					break;
				case 13: //shoot
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(SHOOT_SPEED);
					shooter2->Set(SHOOT_SPEED);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					if(timer->Get() > 0.5)
						autoState++;
					break;
				}
				break;
			}
		}
	}

	//===========================================================TELEOP=======================================
	void TeleopInit(void)
	{
		IMAQdxStartAcquisition(session);
		trimTimer->Start();
		if(m_Joystick->GetRawButton(9))
			m_shooter->SetPosition(0);
		m_LED->Set(Relay::kForward);
		timer->Reset();
		timer->Start();
		time = timer->Get();
		pressTimer->Start();
	}

	void TeleopPeriodic(void)
	{
		operateShifter();
		operateClimber();
		advancedShoot();
		advancedServo();		//shootTemp();
		//simpleShoot();
		//pusher();
		//simpleIntake();
		//SubtractionFilter();
		//if (m_pusherHomeSwitch->Get())
			//m_pusher->Reset();
		//if (m_Joystick->GetRawButton(10))
			//aimsAtTarget();
		if(shooterState != 90){
			FindTargetCenter();
			teleDrive();


		}
		//tempIntake();


		//lw->Run();
		//printf("Robot tele periodic");
	}

	//==========================================================USER FUNCTIONS=================================


	inline void teleDrive(void)
	{


		if(m_Gamepad->GetPOV() == GP_LEFT)
		{
			/*
			printf("timer: %f\t delta: %f", timer->Get(), time - timer->Get());
			if(timer->Get() - time > MAN_AIM_ON + MAN_AIM_OFF)
			{
				time = timer->Get();
			}
			if(timer->Get() - time < MAN_AIM_ON)
			{
				m_leftDrive4->SetSpeed(MAN_AIM_SPEED);
				m_leftDrive1->SetSpeed(MAN_AIM_SPEED);
				m_rightDrive2->SetSpeed(MAN_AIM_SPEED);
				m_rightDrive3->SetSpeed(MAN_AIM_SPEED);
			}

			else if(timer->Get() - time < MAN_AIM_OFF + MAN_AIM_ON)
			{
				m_leftDrive4->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
			}*/

			m_leftDrive4->SetSpeed(MAN_AIM_SPEED);
			m_leftDrive1->SetSpeed(MAN_AIM_SPEED);
			m_rightDrive2->SetSpeed(MAN_AIM_SPEED);
			m_rightDrive3->SetSpeed(MAN_AIM_SPEED);


		}
		else if(m_Gamepad->GetPOV() == GP_RIGHT)
		{
			/*
			printf("timer: %f\t delta: %f", timer->Get(), time - timer->Get());
			if(timer->Get() - time > MAN_AIM_ON + MAN_AIM_OFF)
			{
				time = timer->Get();
			}
			if(timer->Get() - time < MAN_AIM_ON)
			{
				m_leftDrive4->SetSpeed(-MAN_AIM_SPEED);
				m_leftDrive1->SetSpeed(-MAN_AIM_SPEED);
				m_rightDrive2->SetSpeed(-MAN_AIM_SPEED);
				m_rightDrive3->SetSpeed(-MAN_AIM_SPEED);
			}

			else if(timer->Get() - time < MAN_AIM_OFF + MAN_AIM_ON)
			{
				m_leftDrive4->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
			}
			 */
			m_leftDrive4->SetSpeed(-MAN_AIM_SPEED);
			m_leftDrive1->SetSpeed(-MAN_AIM_SPEED);
			m_rightDrive2->SetSpeed(-MAN_AIM_SPEED);
			m_rightDrive3->SetSpeed(-MAN_AIM_SPEED);

		}

		else
		{
			leftSpeed = scale(limit(expo(m_Gamepad2->GetRawAxis(5), 2), 1)  - scale(limit(expo(m_Gamepad2->GetRawAxis(4), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) + scale(limit(expo(m_Joystick->GetY(), 2), 1) - scale(limit(expo(m_Joystick->GetX(), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) + scale(expo(m_Gamepad->GetRawAxis(1), 2), 0.5) - scale(expo(m_Gamepad->GetRawAxis(0), 3), 0.5);
			rightSpeed = scale(-limit(expo(m_Gamepad2->GetRawAxis(5), 2), 1) - scale(limit(expo(m_Gamepad2->GetRawAxis(4), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) + scale(-limit(expo(m_Joystick->GetY(), 2), 1) - scale(limit(expo(m_Joystick->GetX(), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) + scale(expo(-m_Gamepad->GetRawAxis(1), 2), 0.5) - scale(expo(m_Gamepad->GetRawAxis(0), 3), 0.5);

			//printf("Joystick x=%f, y=%f\n", x,y);
			m_leftDrive4->SetSpeed(leftSpeed);
			m_leftDrive1->SetSpeed(leftSpeed);
			m_rightDrive2->SetSpeed(rightSpeed);
			m_rightDrive3->SetSpeed(rightSpeed);

		}
	}


	inline void operateClimber(void)
	{
		if(m_Gamepad->GetRawButton(GP_SELECT) && m_Gamepad->GetRawButton(GP_START))
		{
			m_climbE->Set(false);
			m_climbR->Set(true);
		}
		else
		{
			m_climbE->Set(true);
			m_climbR->Set(false);
		}

		winchSpeed = m_Gamepad->GetRawAxis(5);

		m_climber->Set(winchSpeed);

	}

	/*inline void operatePD(void){
		motorSpeed = -m_Gamepad->GetRawAxis(2) + m_Gamepad->GetRawAxis(3);

		m_PD->Set(motorSpeed);
	}*/

	inline void operateShifter(void){
		if(m_Joystick->GetRawButton(1) || m_Gamepad2->GetRawButton(GP_R)){
			m_shiftHigh->Set(true);
			m_shiftLow->Set(false);
		}
		else{
			m_shiftHigh->Set(false);
			m_shiftLow->Set(true);
		}
	}

	void advancedShoot(void)
	{
		//printf("Shooter State: %d\n", shooterState);
		//printf("gamepad dY: %d\n", m_Gamepad->GetPOV(0));
		if(m_Gamepad->GetRawButton(GP_L) && shooterState != 50)
			m_intakeRoller->SetSpeed(ROLLER_SPEED);
		else if(m_Gamepad->GetRawButton(GP_R) && shooterState != 50)
			m_intakeRoller->SetSpeed(-ROLLER_SPEED);
		else
			m_intakeRoller->SetSpeed(0);

		switch(shooterState)
		{
		case 0:
			//everything is off
			autoShooter(HOME_SHOOTER);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(HOME_INTAKE);
			//m_intakeRoller->SetSpeed(0.f);

			shooterState = 10;
			break;

		case 10:
			//HOME
			autoShooter(HOME_SHOOTER);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(HOME_INTAKE);
			//m_intakeRoller->SetSpeed(0.f);

			/*if (m_Gamepad->GetRawButton(GP_Y))
			{
				shooterState = 49;
				timer->Reset();
				timer->Start();
			}*/
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			else if(m_Gamepad->GetRawButton(GP_B))
				{
					shooterState = 50;
					timer->Reset();
					timer->Start();
				}
			else if(m_Gamepad->GetRawButton(GP_X))
				{
					shooterState = 71;
					timer->Reset();
					timer->Start();
				}
			break;

		/*case 11: //
			autoShooter(HOME_SHOOTER);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(true);
			m_shootR->Set(false);


			autoIntake(HOME_INTAKE);
			//m_intakeRoller->SetSpeed(0.f);


			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			else if(m_Gamepad->GetRawButton(GP_B))
				{
					shooterState = 50;
					timer->Reset();
					timer->Start();
				}
			else if(m_Gamepad->GetRawButton(GP_X))
				{
					shooterState = 71;
					timer->Reset();
					timer->Start();
				}
			break;*/

		case 20:
			//intake to pickup position with ball either in or out
			autoShooter(HOME_SHOOTER);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(PICKUP);
			//m_intakeRoller->SetSpeed(0.f);

			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;

			//else if(m_Gamepad->GetRawButton(GP_R))
				//shooterState = 30;
			//else if(m_Gamepad->GetRawButton(GP_L))
				//shooterState = 31;

			else if(m_Gamepad->GetRawButton(GP_B))
			{
				shooterState = 40;
				timer->Reset();
				timer->Start();
			}
			else if(m_Gamepad->GetRawButton(GP_X))
			{
				shooterState = 71;
				timer->Reset();
				timer->Start();
			}
			break;


		case 40:
			//intake into ball transfer position
			autoShooter(HOME_SHOOTER);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(TRANSFER);
			//m_intakeRoller->SetSpeed(0.f);

			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			else if(m_Gamepad->GetRawButton(GP_B) && timer->Get() > 0.5)
			{
				shooterState = 50;
				timer->Reset();
				timer->Start();

			}
			/*else if(m_Gamepad->GetRawButton(GP_Y) && timer->Get() > 0.5)
			{
				shooterState = 49;
				timer->Reset();
				timer->Start();

			}*/
			else if(m_Gamepad->GetRawButton(GP_X) && timer->Get() > 0.5)
			{
				shooterState = 71;
				timer->Reset();
				timer->Start();

			}
			break;
		case 49: //transfer ball into shooter for shoot close
			autoShooter(HOME_SHOOTER);
			shooter1->Set(SPEED_RPM/6);
			shooter2->Set(-SPEED_RPM/6);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(TRANSFER);
			m_intakeRoller->SetSpeed(-ROLLER_SPEED);

			if(timer->Get() > 0.75)
			{
				shooterState = 61;
				timer->Stop();
				timer->Reset();
			}
			break;



		case 50: //transfer ball into shooter
			autoShooter(HOME_SHOOTER);
			shooter1->Set(SPEED_RPM/6);
			shooter2->Set(-SPEED_RPM/6);
			m_shootE->Set(false);
			m_shootR->Set(true);

			m_intakeRoller->SetSpeed(-ROLLER_SPEED);
			autoIntake(TRANSFER);
			//m_intakeRoller->SetSpeed(-ROLLER_SPEED);

			if(timer->Get() > 0.5)
			{
				shooterState = 60;
				timer->Stop();
				timer->Reset();
			}
			break;

		case 60: //move shooter and intake into shoot far position
			autoShooter(SHOOT_FAR);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			else if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			else if(m_Gamepad->GetRawButton(GP_X))
				{
					shooterState = 70;
					timer->Reset();
					timer->Start();
				}
			else if(m_Gamepad->GetRawButton(GP_A))
			{
				shooterState = 90;
				autoAimAngle = getAutoAimAngle();
				timer->Reset();
				timer->Start();
			}
		/*	else if(m_Gamepad->GetRawButton(GP_Y))
				shooterState = 61;*/
			break;

		case 61: //shoot close position
			autoShooter(SHOOT_CLOSE);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(INTAKE_SHOOT_CLOSE);
			//m_intakeRoller->SetSpeed(0.f);

			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			else if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			else if(m_Gamepad->GetRawButton(GP_X))
				{
					shooterState = 72;
					timer->Reset();
					timer->Start();
				}
			break;

		case 69: //cancel shot
			autoShooter(SHOOT_FAR);
			shooter1->Set(SPEED_RPM/4);
			shooter2->Set(-SPEED_RPM/4);
			m_shootE->Set(false);
			m_shootR->Set(true);

			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			if(timer->Get() > 0.25)
			{
				shooterState = 60;
				timer->Stop();
				timer->Reset();
			}
			break;
		case 70: //prep shooter for shot from far
			autoShooter(SHOOT_FAR);
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM - BALL_SPIN);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			//printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());
			//cylinder = retract|shooter = out|angle = shoot
			//printf("s1: %f\ts2: %f\n", shooter1->GetOutputVoltage(), shooter2->GetOutputVoltage());

			if(m_Gamepad->GetRawButton(GP_B))
			{
				timer->Start();
				timer->Reset();
				shooterState = 69;
			}
			else if(m_Gamepad->GetRawButton(GP_X) && timer->Get() > 0.5 )//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
			{
				timer->Reset();
				timer->Start();
				shooterState = 80;
			}
			else if(m_Gamepad->GetRawButton(GP_A))
			{
				shooterState = 90;
				timer->Reset();
				timer->Start();
			}

			break;

		case 71: //prep shooter for shot from home
			autoShooter(HOME_SHOOTER);
			shooter1->Set(-SPEED_RPM_LOW);
			shooter2->Set(SPEED_RPM_LOW);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			//printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());
			//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
			if(timer->Get() > 0.5)
			{
				timer->Reset();
				timer->Start();
				shooterState = 81;
			}

			break;

		case 72: //prep shooter for shot from batter
			autoShooter(SHOOT_CLOSE);
			shooter1->Set(-SPEED_RPM*0.9);
			shooter2->Set((SPEED_RPM - BALL_SPIN)*0.9);
			m_shootE->Set(false);
			m_shootR->Set(true);

			autoIntake(INTAKE_SHOOT_CLOSE);
			//m_intakeRoller->SetSpeed(0.f);

			//printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());
			//cylinder = retract|shooter = out|angle = shoot
			//printf("s1: %f\ts2: %f\n", shooter1->GetOutputVoltage(), shooter2->GetOutputVoltage());

			if(m_Gamepad->GetRawButton(GP_B))
			{
				timer->Start();
				timer->Reset();
				shooterState = 69;
			}
			else if(m_Gamepad->GetRawButton(GP_X) && timer->Get() > 0.5 )//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
			{
				timer->Reset();
				timer->Start();
				shooterState = 82;
			}

			break;

		case 80: //shoot from shoot_far position
			//printf("Shooting...");
			autoShooter(SHOOT_FAR);
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM - BALL_SPIN);
			m_shootE->Set(true);
			m_shootR->Set(false);

			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			if(timer->Get() > 1.0)
			{
				shooterState = 10;
				timer->Stop();
				timer->Reset();
			}
			break;

		case 81:
			//shoot from home
			//printf("Shooting...");
			autoShooter(HOME_SHOOTER);
			shooter1->Set(-SPEED_RPM_LOW);
			shooter2->Set(SPEED_RPM_LOW);
			m_shootE->Set(true);
			m_shootR->Set(false);

			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			if(timer->Get() > 0.5)
			{
				shooterState = 10;
				timer->Stop();
				timer->Reset();
			}
			break;
		case 82:
			//shoot from batter
			//printf("Shooting...");
			autoShooter(SHOOT_CLOSE);
			shooter1->Set(-SPEED_RPM*0.9);
			shooter2->Set((SPEED_RPM - BALL_SPIN)*0.9);
			m_shootE->Set(true);
			m_shootR->Set(false);

			autoIntake(INTAKE_SHOOT_CLOSE);
			//m_intakeRoller->SetSpeed(0.f);
			printf("timer is %f\n", timer->Get());
			if(timer->Get() > 1.0)
			{
				shooterState = 10;
				timer->Stop();
				timer->Reset();
			}
			break;
		case 89:
			//printf("Shooting...");
			//autoShooter(findShooterAngle());
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM - BALL_SPIN);
			m_shootE->Set(true);
			m_shootR->Set(false);

			autoShooter(SHOOT_FAR);
			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			if(timer->Get() > 1.0)
			{
				shooterState = 10;
				timer->Stop();
				timer->Reset();
			}
			break;
		case 90: //vision
			FindTargetCenter();
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM - BALL_SPIN);
			autoShooter(SHOOT_FAR);
			autoIntake(INTAKE_SHOOT_FAR);
			if(!m_Gamepad->GetRawButton(GP_A))
				shooterState = 70;
			if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 0.5){//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){//goal object detected
				timer->Reset();
				timer->Start();
				shooterState = 89;
			}
			break;
		case 91: //spin up
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM - BALL_SPIN);
			autoShooter(SHOOT_FAR);
			autoIntake(INTAKE_SHOOT_FAR);
			if(timer->Get() > 0.5)
			{
				timer->Reset();
				timer->Start();
				autoState = 89;
			}
			else if(m_Gamepad->GetRawButton(GP_B))
			{
				timer->Start();
				timer->Reset();
				shooterState = 69;
			}
			break;
		}
	}

	inline void shootTemp()
	{
		switch(shooterState1)
		{
		case 0:
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);
			if(m_Gamepad->GetRawButton(GP_X))
				{
					timer->Reset();
					timer->Start();
					autoState++;
				}
			break;
		case 1: //prep shooter for shot from batter
			shooter1->Set(-SPEED_RPM);
			shooter2->Set((SPEED_RPM - BALL_SPIN));
			m_shootE->Set(false);
			m_shootR->Set(true);

			//m_intakeRoller->SetSpeed(0.f);

			//printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());
			//cylinder = retract|shooter = out|angle = shoot
			//printf("s1: %f\ts2: %f\n", shooter1->GetOutputVoltage(), shooter2->GetOutputVoltage());

			if(m_Gamepad->GetRawButton(GP_X) && timer->Get() > 0.5 )//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
			{
				timer->Reset();
				timer->Start();
				shooterState++;
			}
			else if(m_Gamepad->GetRawButton(GP_B))
				autoState = 0;
			break;

		case 2: //shoot from shoot_far position
			//printf("Shooting...");
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM - BALL_SPIN);
			m_shootE->Set(true);
			m_shootR->Set(false);

			//m_intakeRoller->SetSpeed(0.f);

			if(timer->Get() > 1.0)
			{
				shooterState = 0;
				timer->Stop();
				timer->Reset();
			}
			break;
		}

	}

	inline void simpleIntake()
	{
		if(m_Joystick->GetRawButton(3))
			m_intake->SetSetpoint(0.45);
		else if(m_Joystick->GetRawButton(5))
			m_intake->SetSetpoint(-0.45);
		else
			m_intake->SetSetpoint(0.f);
		if(m_Joystick->GetRawButton(8))
			m_intakeRoller->SetSpeed(0.6);
		else if(m_Joystick->GetRawButton(9))
			m_intakeRoller->SetSpeed(-0.6);
		else
			m_intakeRoller->SetSpeed(0.f);
	}

	bool autoShooter(int ticks)
	{
		//shooterPID->setMaxOutput(0.2);
		int currentTicks = m_shooter->GetEncPosition();

		shooterPID->setDesiredValue(ticks);

		float rotate = shooterPID->calcPID(currentTicks);

		m_shooter->Set(-rotate);

		//printf("shooter rotate power: %f \t error: %d \t target: %d \n", rotate, currentTicks - ticks, ticks);

		return shooterPID->isDone();
	}

	bool autoIntake(int ticks)
		{
			//intakePID->setMaxOutput(0.2);
			int currentTicks = m_intake->GetEncPosition();

			intakePID->setDesiredValue(ticks);

			float rotate = intakePID->calcPID(currentTicks);

			m_intake->Set(-rotate);

		//printf("intake rotate power: %f \t error: %d \t target: %d \n", rotate, currentTicks - ticks, ticks);

			return intakePID->isDone();
		}


	bool autoDrive(int distance, int angle)
	{
		int currentDist = (m_rightDriveEncoder->Get() + m_leftDriveEncoder->Get()) / 2;
		int currentAngle = nav->GetYaw();

		drivePID->setDesiredValue(distance);
		turnPID->setDesiredValue(angle);

		float drive = -drivePID->calcPID(currentDist);
		float turn = -turnPID->calcPID(currentAngle);

		m_rightDrive2->SetSpeed(-(limit(drive - turn, 1)));
		m_rightDrive3->SetSpeed(-(limit(drive - turn, 1)));
		m_leftDrive4->SetSpeed(limit(drive + turn, 1));
		m_leftDrive1->SetSpeed(limit(drive + turn, 1));

		return drivePID->isDone() && turnPID->isDone();
	}



/*	void updatePosition(void)
	{
		float currentAngle = nav->GetYaw();
		int currentX = (m_rightDriveEncoder->Get() + m_leftDriveEncoder->Get())/2*cos(currentAngle);
		int currentY =
	}

	bool advancedAutoDrive()
	{

	}*/

	void advancedServo(void)
	{
		switch(servoState)
		{
		case 0:
			m_PDServo1->SetAngle(SERVO_IN_1);
			m_PDServo2->SetAngle(SERVO_IN_2);
			if(m_Gamepad->GetRawButton(GP_Y) && pressTimer->Get() > 0.5)
			{
				servoState++;
				pressTimer->Reset();
			}
			break;
		case 1:
			m_PDServo1->SetAngle(SERVO_OUT_1);
			m_PDServo2->SetAngle(SERVO_OUT_2);
			if(m_Gamepad->GetRawButton(GP_Y) && pressTimer->Get() > 0.5)
			{
				servoState--;
				pressTimer->Reset();
			}
			break;
		}
	}

	//===============================================VISION FUNCTIONS=============================================

	float getAutoAimAngle(){
		int image_error;
		image_error = FindTargetCenter();

#ifdef SAVE_SHOT_PICTURES
		sprintf(filename, "/home/lvuser/pic%d.bmp", picture_ID);
		DriverStation::ReportError("writing shot picture " + std::to_string((int)picture_ID) + " to file\n");
		imaqWriteBMPFile(frame, filename, 30, &colourTable);
		picture_ID++;
#endif

		//nav->Reset();
		if (image_error == 0){
			float angle = atan((centerx - (RES_X/2) - AIM_CORRECTION)*tan(HFOV)/(RES_X/2))*180/PI + AUTO_AIM_CORRECTION;
			printf("\t\tCalced angle from bot %f\n", angle);
			return normal(nav->GetYaw() + angle);
		}
		else
			return image_error;
	}

	int aimAtTarget2(float angle){
		int currentAngle = nav->GetYaw();
		turnPID2->setDesiredValue(angle);
		float turn = -turnPID2->calcPID(currentAngle);
		m_rightDrive2->SetSpeed(turn);
		m_rightDrive3->SetSpeed(turn);
		m_leftDrive4->SetSpeed(turn);
		m_leftDrive1->SetSpeed(turn);
		printf("aiming at target %f from %f\n", angle, nav->GetYaw());
		return turnPID2->isDone();
	}

	int aimAtTarget(void){
		float turn = last_turn;
		int image_error;
		float error, humanAdjust;

		//take a picture after some time
		if(aim_loop_counter >= AIM_LOOP_WAIT){
			image_error = FindTargetCenter();
			//if the target was found calculate turn speed
			if (image_error == 0) {
				visionPID->setDesiredValue(IMAGE_CENTER + AIM_CORRECTION);
				turn = visionPID->calcPID(centerx);
				turn = AIM_FILTER*(turn - last_turn) + last_turn;
				error = IMAGE_CENTER + AIM_CORRECTION - centerx;
				last_turn = turn;
				aim_loop_counter = 0;
			}
			//printf("im_error: %d\tcenter: %f\terror: %f\tturn: %f\n", image_error, centerx, error, turn);

			//SmartDashboard::PutNumber("Aim Error", IMAGE_CENTER + AIM_CORRECTION - x_pixel);
			//SmartDashboard::PutNumber("Motor Output", turn);
		}

		/*
		if (target_y < CLOSE_LIMIT)
			autoShooter(SHOOT_CLOSE);
		else
			autoShooter(SHOOT_FAR);
		*/

		//if image hasn't been found in a while then stop moving
		if (aim_loop_counter - AIM_LOOP_WAIT >= AIM_TIMEOUT){
			turn = last_turn = 0;
		}

		aim_loop_counter++;

		humanAdjust = scale(m_Gamepad->GetRawAxis(0), 0.5);
		turn -= humanAdjust;

		if (m_Joystick->GetRawButton(7) && trimTimer->Get() > 0.5){
			trimTimer->Reset();
			aim_fly_trim += 1.0f;
			printf("\t\taim trim: %f\n", aim_fly_trim);
		}
		else if (m_Joystick->GetRawButton(8) && trimTimer->Get() > 0.5){
			trimTimer->Reset();
			aim_fly_trim -= 1.0f;
			printf("\t\taim trim: %f\n", aim_fly_trim);
		}


		if(error < AIM_FINE_LIMIT && error > -AIM_FINE_LIMIT){
			m_leftDrive4->SetSpeed(turn/2.f);
			m_leftDrive1->SetSpeed(turn/2.f);
			m_rightDrive2->SetSpeed(turn*1.25f);
			m_rightDrive3->SetSpeed(turn*1.25f);
		}
		else{
			m_leftDrive4->SetSpeed(turn);
			m_leftDrive1->SetSpeed(turn);
			m_rightDrive2->SetSpeed(turn);
			m_rightDrive3->SetSpeed(turn);
		}


		if(visionPID->isDone() && image_error == 0){
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

			if(m_Joystick->GetRawButton(12)){
				sprintf(filename, "/home/lvuser/pic%d.bmp", picture_ID);
				DriverStation::ReportError("writing picture to file\n");
				imaqWriteBMPFile(frame, filename, 30, &colourTable);
				picture_ID++;
			}

			//find filtered blobs
			imaqParticleFilter4(particle, processed, filterCriteria, CRITERIA_COUNT, &filterOptions, NULL, &num_particlesFound);


			if (num_particlesFound == 0){
				//unable to find target
				DriverStation::ReportError("ERROR! Target not found!\n");
				imaqDrawShapeOnImage(frame, frame, {0, IMAGE_CENTER+AIM_CORRECTION, RES_Y-1, 1}, IMAQ_DRAW_INVERT, IMAQ_SHAPE_RECT, 255.0f);
				if (m_Joystick->GetRawButton(11))
					CameraServer::GetInstance()->SetImage(processed);
				else
					CameraServer::GetInstance()->SetImage(frame);
				return -3;
			}
			else if (num_particlesFound > 0){
				//take measurements
				int blob = pickBlob(num_particlesFound);
				imaqMeasureParticle(particle, blob, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &rect_left);
				imaqMeasureParticle(particle, blob, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &rect_width);
				imaqMeasureParticle(particle, blob, FALSE, IMAQ_MT_BOUNDING_RECT_TOP, &target_y);

				//showBlobMeasurements();

				//find center based on width
				centerx = rect_left + (rect_width/2);

				//optional draw circle, or reference lines for visual confirmation
				//imaqDrawShapeOnImage(processed, processed, {(int)(target_y - (rect_width/2.f)), (int)(centerx - (rect_width/2.f)), (int)rect_width, (int)rect_width}, IMAQ_DRAW_INVERT,IMAQ_SHAPE_OVAL,0);
				imaqDrawShapeOnImage(processed, processed, {0, IMAGE_CENTER+AIM_CORRECTION, RES_Y-1, 1}, IMAQ_DRAW_INVERT, IMAQ_SHAPE_RECT, 0);
				imaqDrawShapeOnImage(processed, processed, {(int)(target_y - rect_width/2.f), (int)centerx, (int)(rect_width/2), 1}, IMAQ_DRAW_INVERT, IMAQ_SHAPE_RECT, 0);
				SmartDashboard::PutNumber("target center", centerx);
				//printf("target width, x, y: %f\t%f\t%f\n", rect_width, centerx, target_y);
			}

			if (m_Joystick->GetRawButton(11))
				CameraServer::GetInstance()->SetImage(frame);
			else
				CameraServer::GetInstance()->SetImage(processed);


			return 0;
		}
		// unable to take picture, return error
		DriverStation::ReportError("ERROR! Unable to take picture!\n");
		return -1;
	}

	int pickBlob(int num_blobs)
	{
		double width[num_blobs], left[num_blobs], area[num_blobs], perimeter[num_blobs], height[num_blobs], aspect[num_blobs], fill[num_blobs];
		double maxArea = 0, maxAreaIndex = 0;
		for (int i = 0; i < num_particlesFound; i++)
		{
			imaqMeasureParticle(particle, i , FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &width[i]);
			imaqMeasureParticle(particle, i , FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &left[i]);
			imaqMeasureParticle(particle, i , FALSE, IMAQ_MT_AREA, &area[i]);
			imaqMeasureParticle(particle, i , FALSE, IMAQ_MT_PERIMETER, &perimeter[i]);
			imaqMeasureParticle(particle, i , FALSE, IMAQ_MT_BOUNDING_RECT_HEIGHT, &height[i]);

			aspect[i] = height[i] / width[i];
			fill[i] = area[i] / width[i] * height[i];

			if (area[i] > maxArea && area[i] > 1000)
			{
				maxArea = area[i];
				maxAreaIndex = i;
			}
		}
		//printf("picking blob %d\n", (int)maxAreaIndex);
		return (int)maxAreaIndex;
	}

	void showBlobMeasurements(void){
		double width, left, area, perimeter, height, aspect, fill;
		for (int i = 0; i < num_particlesFound; i++)
		{
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &width);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &left);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_AREA, &area);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_PERIMETER, &perimeter);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_HEIGHT, &height);

			aspect = height / width;
			fill = area / (width *height);

			printf("blob%d width: %d\tleft: %d\tarea: %d\tperimeter: %d\theight: %d, aspect: %f, fill: %f\n", i, (int)width, (int)left,
					(int)area, (int)perimeter, (int)height, (float)aspect, (float)fill);

		}
		printf("width: %d\tleft: %d\tarea: %d\tperimeter: %d\theight: %d\n", (int)width, (int)left, (int)area, (int)perimeter, (int)height);
	}

	inline void BinaryFilter(void){
		for (int i = 0; i < (RES_X*RES_Y); i++){
			/*
			int x = i % RES_X;
			int y = (int)(i / RES_X);
			if (y > 400)
				printf("R, G, B: (%d, %d, %d), (%d, %d)\n", R[i*4], G[i*4], B[i*4], x, y);
			*/

			if ((G[i*4] > G_THRESHOLD)){
				//printf("got green pixel: %d\n", G[i*4]);
				proc_pixel[i] = 255;
			}
			else
				proc_pixel[i] = 0;


			//if it has lots of red
			if ((R[i*4] > R_THRESHOLD)){
				proc_pixel[i] = 0;
				//printf("%d too red\n", i);
			}
			//blue without green
			else if (((B[i*4] > B_THRESHOLD) && (G[i*4] < G_THRESHOLD))){
				//printf("eliminating pixel BG: %d, %d\n", B[i*4], G[i*4]);
				proc_pixel[i] = 0;
				//printf("%d too blue no green", i);

			}
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

	inline int findShooterAngle()
	{
		int angle = (SLOPE*target_y + INTERCEPT + SHOOTER_TRIM + aim_fly_trim)*(4096.f/360.f);
		//printf("auto shooter angle: %d\n", angle);
		return angle;
	}

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
	inline float normal(float x)
	{
		if(x > 180)
			x -= 360;
		else if(x < -180)
			x += 360;
		return x;
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
