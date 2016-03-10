/**
 * Changelog: please comment your commit message
 * - LW
 * March 10
 * Everything untested
 * Angle (roll) has to be between -10 and 10 in auto modes 3 and 4 to shoot (prevents misfire if robot caught up on defense)
 * Added extra drive to auto position 5 to get the robot in front of of the middle goal
 * Increased drivePID max output to 1.0
 * Added teleDriveAlt() to give operator full drive control
 * Added 2-ball auto (8)
 */


#include "WPILib.h"
#include "NIIMAQdx.h"
#include <math.h>
#include "SimPID.h"
#include "AHRS.h"

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
#define GP_Y 4
#define GP_L 5
#define GP_R 6
#define OPEN 1
#define CLOSED 0
#define SERVO_IN 90
#define SERVO_OUT 45

#define SHOOT_SPEED 0.8225f
#define PICKUP 1770
#define SHOOT_LOWBAR 390
#define SHOOT_FAR 480
#define SHOOT_CLOSE 550
#define INTAKE_SHOOT_FAR 550
#define INTAKE_SHOOT_CLOSE 475
#define TRANSFER 0
#define HOME_SHOOTER 0
#define HOME_INTAKE 0
#define SPEED_RPM 7000
#define SHOOTER_SPEED_CHECK 20000

//vision
#define AIM_CORRECTION 45
#define AIM_FILTER 1
#define AIM_LOOP_WAIT 5
#define AIM_TIMEOUT 2
#define AIM_FINE_LIMIT 20
#define CLOSE_LIMIT 220
#define IMAGE_CENTER 320

#define R_THRESHOLD 200
#define G_THRESHOLD 100
#define B_THRESHOLD 200

#define SLOPE -0.0779f
#define INTERCEPT 58.699f
#define SHOOTER_TRIM 1.f

#define TRIM_MODE

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();

	int autoState, autoMode, autoPosition, shooterState, pusherState, getDistance;

	Victor *m_leftDrive4; //0
	Victor *m_leftDrive1; //1
	Victor *m_rightDrive2; //2
	Victor *m_rightDrive3; //3
	float leftSpeed, rightSpeed;

	VictorSP *m_pusher;
	VictorSP *m_intakeRoller;

	CANTalon *shooter1, *shooter2, *m_shooter, *m_intake;

	PowerDistributionPanel *PDP;

	Relay *m_LED;

	Solenoid *m_shiftHigh, *m_shiftLow;
	Solenoid *m_shootE, *m_shootR;

	Servo *m_shooterServo;

	SimPID *drivePID;
	SimPID *turnPID;
	SimPID *visionPID;
	SimPID *intakePID;
	SimPID *shooterPID;
	SimPID *turnPID2;

	Joystick *m_Gamepad;
	Joystick *m_Joystick;

	Timer *timer;
	Timer *trimTimer;
	Timer *stateTimer;
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


	//vision filter options
	ParticleFilterOptions2 filterOptions;
#define CRITERIA_COUNT 4
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

		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
		m_shootE = new Solenoid(3);
		m_shootR = new Solenoid(2);

		m_shooterServo = new Servo(6);

		m_Joystick = new Joystick(0);
		m_Gamepad = new Joystick(1);

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

		m_intakeRoller = new VictorSP(7);

		m_LED = new Relay(0);

		drivePID = new SimPID(0.0005 ,0, 0, 500);
		drivePID->setMinDoneCycles(1);
		drivePID->setMaxOutput(1.0);

		turnPID = new SimPID(0.1, 0, 0, 2);
		turnPID->setMinDoneCycles(1);
		turnPID->setMaxOutput(0.5);

		turnPID2 = new SimPID(0.08,0,0.01,1);
		turnPID2->setMinDoneCycles(10);
		turnPID2->setMaxOutput(0.5);

		shooterPID = new SimPID(0.00175, 0, 0.0005, 10);
		shooterPID->setMaxOutput(0.25);

		intakePID = new SimPID(0.001, 0, 0.001, 10);
		intakePID->setMaxOutput(0.5);

		visionPID = new SimPID(0.01, 0.02, 0.001, 5);
		visionPID->setMaxOutput(0.5);
		visionPID->setMinDoneCycles(20);

		m_shooterHomeSwitch = new DigitalInput(1);
		m_intakeHomeSwitch = new DigitalInput(0);

		m_leftDriveEncoder = new Encoder(4, 5);
		m_rightDriveEncoder = new Encoder(7,6);

		m_shooter = new CANTalon(2);
		m_shooter->SetFeedbackDevice(CANTalon::QuadEncoder);
		//m_shooter->ConfigEncoderCodesPerRev(4096);
		m_shooter->SetSensorDirection(true);

		m_intake = new CANTalon(4);
		//m_intake->SetFeedbackDevice(CANTalon::QuadEncoder);
		m_intake->ConfigEncoderCodesPerRev(4096);

		nav = new AHRS(SPI::Port::kMXP);

		m_pusher = new VictorSP(5);

		PDP = new PowerDistributionPanel(0);

		timer = new Timer();
		timer->Reset();
		stateTimer = new Timer();
		stateTimer->Reset();
		trimTimer = new Timer();
		trimTimer->Reset();

		VisionInit();

		shooterState = 0;
		autoState = 0;
		autoMode = 0;
		autoPosition = 0;
		pusherState = 0;
		aim_fly_trim = 0;
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
		R = raw_pixel;
		G = raw_pixel + 1;
		B = raw_pixel + 2;
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

		for (int i = 1; i <= 12; i++)
		{
			if(m_Joystick->GetRawButton(i))
			{
				if (i < 8)
					autoMode = i;
				else if (i >= 8 && i <= 12)
					autoPosition = i - 7;

				nav->Reset();
				autoDelay = -5*(m_Joystick->GetRawAxis(3)) + 5;
				m_leftDriveEncoder->Reset();
				m_rightDriveEncoder->Reset();
				DriverStation::ReportError("Auto mode: " + std::to_string((long)autoMode) + " position: " + std::to_string((long)autoPosition) + "\n" + "autoDelay" + std::to_string((float)autoDelay));
			}
		}


		DriverStation::ReportError("Gyro: " + std::to_string((float)nav->GetYaw()) +
				" enc: " + std::to_string((long)m_leftDriveEncoder->Get()) +
				", " + std::to_string((long)m_rightDriveEncoder->Get()) + "\n");



		//printf("shooterA: %d\tintakeA: %d\n", m_shooter->GetEncPosition(), m_intake->GetEncPosition());
		//printf("shooterA: %f\tintakeA: %f\n", m_shooter->GetPosition(), m_intake->GetPosition());

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
			//stateTimer->Reset();
			stateTimer->Start();
			IMAQdxStartAcquisition(session);
			m_LED->Set(Relay::kForward);
		}

		void AutonomousPeriodic(void)
		{
#define AUTO_OVER_MOAT -15250
#define AUTO_OVER_OTHER -13000
#define AUTO_LOWBAR_DRIVE -12000
#define AUTO_POS_2_EXTRA_DRIVE -15000
#define AUTO_AIM_POS_1 50
#define AUTO_AIM_POS_2 -25
#define AUTO_AIM_POS_3 10
#define AUTO_AIM_POS_4 -5
#define AUTO_AIM_POS_5 -45
#define AUTO_SHOOTER_POS SHOOT_FAR+40
#define ROLLER_SPEED -0.6

			if(stateTimer->Get() > autoDelay)
			{

			switch(autoMode)

			{
			case 0:
				m_leftDrive4->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);

				autoShooter(HOME_SHOOTER);
				autoIntake(HOME_INTAKE);

				m_intakeRoller->SetSpeed(0.f);

				break;
			case 1: //drive forward with boulder preloaded and move under low bar
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

			case 2: //drive forward with boulder preloaded, cross low bar, discharge ball to tower, revere to neutral zone through low bar, ready to get next boulder
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
					if(timer->Get() > 1.0)
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
					if(autoDrive(16000, 0))
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

					if(autoDrive(19000, 50))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 4: //discharge ball
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
				case 5: //turn around
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					if(autoDrive(19000, -130))
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 6: //drive a little
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					if(autoDrive(21500, -130))
					{
						autoState++;
					}
					break;
				case 7: //turn towards lowbar
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					if(autoDrive(21500, -180))
						autoState++;
					break;
				case 8: //return under lowbar
					autoShooter(HOME_SHOOTER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);

					autoIntake(PICKUP);
					m_intakeRoller->SetSpeed(0.f);

					autoDrive(37500, -180);
					break;
				}
				break;

			case 3: // drive over flat defense in any position and shoot high goal
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
						result = autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_2);
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
					}
					if(result && timer->Get() > 1.5)
					{
						autoAimAngle = getAutoAimAngle();
						if(autoPosition == 2)
							{
								timer->Reset();
								timer->Start();
								autoState = 6;
							}
						else if(autoPosition == 5)
							{
								timer->Reset();
								timer->Start();
								autoState = 8;
							}
						else
							autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 3: //prep ball 2, confirm aim with vision
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM);
					autoShooter(findShooterAngle());
					autoIntake(INTAKE_SHOOT_FAR);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 0.75){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 4: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM);
					autoShooter(findShooterAngle());
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//m_shooterServo->SetAngle(SERVO_OUT);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.3)
					{
						autoState++;
						timer->Stop();
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
					//m_shooterServo->SetAngle(SERVO_IN);
					m_intakeRoller->SetSpeed(0.f);
					break;
				case 6: //position 2 extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(-15000, -25))
						{
							timer->Stop();
							timer->Reset();
							autoState++;
						}


					break;
				case 7: //position 2 rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(-16000, 35))
						{
							autoAimAngle = getAutoAimAngle();
							timer->Stop();
							timer->Reset();
							autoState = 3;
						}
					break;
				case 8: //position 5 extra drive
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(-14000, -45))
						{
							timer->Stop();
							timer->Reset();
							autoState++;
						}


					break;
				case 9: //position 5 rough angle
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					//m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(-14000, 0))
						{
							autoAimAngle = getAutoAimAngle();
							timer->Stop();
							timer->Reset();
							autoState = 3;
						}
					break;
				}
				break;

				case 4: // drive over flat defense in any position and shoot high goal
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
							result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_2);
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
						}
						if(result && timer->Get() > 1.5)
						{
							autoAimAngle = getAutoAimAngle();
							if(autoPosition == 2)
								{
									timer->Reset();
									timer->Start();
									autoState = 6;
								}
							else if(autoPosition == 5)
								{
									timer->Reset();
									timer->Start();
									autoState = 8;
								}
							else
								autoState++;
							timer->Reset();
							timer->Start();
						}
						break;
					case 3: //prep ball 2, confirm aim with vision
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM);
						autoShooter(findShooterAngle());
						autoIntake(INTAKE_SHOOT_FAR);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 0.75){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
							autoState++;
							timer->Reset();
							timer->Start();
						}
						break;
					case 4: //shoot the ball
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM);
						autoShooter(findShooterAngle());
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(true);
						m_shootR->Set(false);
						//m_shooterServo->SetAngle(SERVO_OUT);
						m_intakeRoller->SetSpeed(0.f);
						if(timer->Get() > 0.3)
						{
							autoState++;
							timer->Stop();
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
						//m_shooterServo->SetAngle(SERVO_IN);
						m_intakeRoller->SetSpeed(0.f);
						break;
					case 6: //position 2 extra drive
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(-17250, -25))
							{
								timer->Stop();
								timer->Reset();
								autoState++;
							}


						break;
					case 7: //position 2 rough angle
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(-16000, 35))
							{
								autoAimAngle = getAutoAimAngle();
								timer->Stop();
								timer->Reset();
								autoState = 3;
							}
						break;
					case 8: //position 5 extra drive
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(-16250, -45))
							{
								timer->Stop();
								timer->Reset();
								autoState++;
							}


						break;
					case 9: //position 5 rough angle
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(-14000, 0))
							{
								autoAimAngle = getAutoAimAngle();
								timer->Stop();
								timer->Reset();
								autoState = 3;
							}
						break;
					}
					break;



			case 5: //under lowbar, shoot with vision high goal
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
					case 1: //intake down
						autoShooter(HOME_SHOOTER);
						autoIntake(PICKUP);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_shootE->Set(true);
						m_shootR->Set(false);
						//m_shooterServo->SetAngle(SERVO_IN);
						if(timer->Get() > 1.0)
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
						m_shootE->Set(true);
						m_shootR->Set(false);
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
						if(timer->Get() > 1.5)
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
						autoShooter(findShooterAngle());
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN)
						autoState++;
						break;
					case 7: //confirm aim with vision
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
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
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
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
					case 9: //turn off shooter
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						break;
					}
					break;

				case 7: // drive over cheval, shoot in high goal
					printf("autoState: %d\n", autoState);
					switch(autoState){
					case 0:
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						autoState++;
						break;
					case 1: //drive at defense
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						m_leftDrive4->SetSpeed(-0.5);
						m_leftDrive1->SetSpeed(-0.5);
						m_rightDrive2->SetSpeed(0.5);
						m_rightDrive3->SetSpeed(0.5);
						if(nav->GetRoll() < -8)
						{
							getDistance = (m_leftDriveEncoder->Get() + m_rightDriveEncoder->Get()) / 2;
							m_leftDrive4->SetSpeed(0.f);
							m_leftDrive1->SetSpeed(0.f);
							m_rightDrive2->SetSpeed(0.f);
							m_rightDrive3->SetSpeed(0.f);
							timer->Start();
							autoState++;
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
						if(timer->Get() > 3)
						{
							timer->Stop();
							timer->Reset();
							autoState++;
						}
						break;
					case 3: //drive to OZ
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						if(autoDrive(-AUTO_OVER_OTHER - getDistance, 0))
						{
							timer->Start();
							autoState++;
						}
						break;
					case 4: //aim at tower roughly and prep ball 1
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
							result = autoDrive(-AUTO_OVER_MOAT, AUTO_AIM_POS_2);
							break;
						case 3:
							result = autoDrive(-AUTO_OVER_MOAT, AUTO_AIM_POS_3);
							break;
						case 4:
							result = autoDrive(-AUTO_OVER_MOAT, AUTO_AIM_POS_4);
							break;
						case 5:
							result = autoDrive(-AUTO_OVER_MOAT, AUTO_AIM_POS_5);
							break;
						}
						if(result && timer->Get() > 1.5)
						{
							autoAimAngle = getAutoAimAngle();
							if(autoPosition == 2)
								{
									timer->Reset();
									timer->Start();
									autoState = 6;
								}
							else
								autoState++;
							timer->Reset();
							timer->Start();
						}
						break;
					case 5: //prep ball 2, confirm aim with vision
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM);
						autoShooter(findShooterAngle());
						autoIntake(INTAKE_SHOOT_FAR);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 0.75){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
							autoState++;
							timer->Reset();
							timer->Start();
						}
						break;
					case 6: //shoot the ball
						shooter1->Set(-SPEED_RPM);
						shooter2->Set(SPEED_RPM);
						autoShooter(findShooterAngle());
						autoIntake(INTAKE_SHOOT_FAR);
						m_shootE->Set(true);
						m_shootR->Set(false);
						//m_shooterServo->SetAngle(SERVO_OUT);
						m_intakeRoller->SetSpeed(0.f);
						if(timer->Get() > 0.3)
						{
							autoState++;
							timer->Stop();
							timer->Reset();
						}
						break;
					case 7: //turn off shooter
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						autoShooter(HOME_SHOOTER);
						autoIntake(HOME_INTAKE);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						m_intakeRoller->SetSpeed(0.f);
						break;
					case 8: //position 2 extra drive
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(-17250, -25))
							{
								timer->Stop();
								timer->Reset();
								autoState++;
							}


						break;
					case 9: //position 2 rough angle
						autoShooter(SHOOT_FAR);
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(0.f);
						shooter2->Set(0.f);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN);
						if (autoDrive(-15000, 35))
							{
								autoAimAngle = getAutoAimAngle();
								timer->Stop();
								timer->Reset();
								autoState = 3;
							}
						break;
					}
					break;
				case 8: //2-ball auto
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
						if(timer->Get() > 1.5)
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
						autoShooter(findShooterAngle());
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN)
						autoState++;
						break;
					case 7: //confirm aim with vision
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
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
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
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
						autoShooter(findShooterAngle());
						autoIntake(INTAKE_SHOOT_FAR);
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
						m_intakeRoller->SetSpeed(0.f);
						m_shootE->Set(false);
						m_shootR->Set(true);
						//m_shooterServo->SetAngle(SERVO_IN)
						autoState++;
						break;
					case 16: //confirm aim with vision
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
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
						shooter1->Set(SPEED_RPM);
						shooter2->Set(-SPEED_RPM);
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
	}

	void TeleopPeriodic(void)
	{
		operateShifter();
		advancedShoot();
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

#define PRACTICE_DRIVE_LIMIT 1
	inline void teleDrive(void)
	{
		leftSpeed = scale(limit(expo(m_Joystick->GetY(), 2), 1) + scale(m_Gamepad->GetRawAxis(1), 0.5) - scale(limit(expo(m_Joystick->GetX(), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) - scale(m_Gamepad->GetRawAxis(0), 0.5);
		rightSpeed = scale(-limit(expo(m_Joystick->GetY(), 2), 1) + scale(-m_Gamepad->GetRawAxis(1), 0.5) - scale(limit(expo(m_Joystick->GetX(), 3), 1), 0.7f), PRACTICE_DRIVE_LIMIT) - scale(m_Gamepad->GetRawAxis(0), 0.5);

		//printf("Joystick x=%f, y=%f\n", x,y);
		m_leftDrive4->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);
	}


	inline void operateShifter(void){
		if(m_Joystick->GetRawButton(1)){
			m_shiftHigh->Set(true);
			m_shiftLow->Set(false);
		}
		else{
			m_shiftHigh->Set(false);
			m_shiftLow->Set(true);
		}
	}

	inline void simpleShoot(void){
		if (m_Gamepad->GetRawButton(GP_Y)){
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
			m_pusher->Set(PUSHER_SPEED);
		else if (m_Joystick->GetRawButton(4))
			m_pusher->Set(-PUSHER_SPEED);
		else
			m_pusher->Set(0.f);
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

			if (m_Gamepad->GetRawButton(GP_Y))
				shooterState = 11;
			else if(m_Gamepad->GetPOV() == GP_UP)
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

		case 11: //HOME travel
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
			break;

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

		/*case 30:
			//Ball into intake
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_intakeRoller->SetSpeed(-0.6);
			autoShooter(HOME_SHOOTER);
			autoIntake(PICKUP);
			m_shootE->Set(false);
			m_shootR->Set(true);
			if(!m_Gamepad->GetRawButton(GP_R))
				shooterState = 20;
			break;
		case 31:
			//Spit ball out of intake
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_intakeRoller->SetSpeed(0.6);
			autoShooter(HOME_SHOOTER);
			autoIntake(PICKUP);
			m_shootE->Set(false);
			m_shootR->Set(true);
			if(!m_Gamepad->GetRawButton(GP_L))
				shooterState = 20;
			break;
			*/
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
			break;

		case 50: //transfer ball into shooter
			autoShooter(HOME_SHOOTER);
			shooter1->Set(SPEED_RPM/6);
			shooter2->Set(-SPEED_RPM/6);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(TRANSFER);
			m_intakeRoller->SetSpeed(-ROLLER_SPEED);

			if(timer->Get() > 0.75)
			{
				shooterState = 60;
				timer->Stop();
				timer->Reset();
			}
			break;

		case 60: //move shooter and intake into shoot position
			autoShooter(SHOOT_FAR);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			else if(m_Gamepad->GetRawButton(GP_X) || m_Gamepad->GetRawButton(GP_A))
				{
					shooterState = 70;
					autoAimAngle = getAutoAimAngle();
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

		case 70: //prep shooter for shot
			autoShooter(SHOOT_FAR);
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
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
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
			m_shootE->Set(false);
			m_shootR->Set(true);


			autoIntake(INTAKE_SHOOT_FAR);
			//m_intakeRoller->SetSpeed(0.f);

			//printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());

			if(m_Gamepad->GetRawButton(GP_B))
			{
				timer->Start();
				shooterState = 10;
			}
			else if(m_Gamepad->GetRawButton(GP_X) && timer->Get() > 1.0 )//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
			{
				timer->Reset();
				timer->Start();
				shooterState = 81;
			}

			break;

		case 80: //shoot from shoot_far position
			//printf("Shooting...");
			autoShooter(SHOOT_FAR);
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
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
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
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

		case 89:
			//printf("Shooting...");
			autoShooter(findShooterAngle());
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
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
		case 90: //vision
			FindTargetCenter();
			autoShooter(SHOOT_FAR);
			autoIntake(INTAKE_SHOOT_FAR);
			if(!m_Gamepad->GetRawButton(GP_A))
				shooterState = 70;
			if(aimAtTarget2(autoAimAngle) == 1 && timer->Get() > 0.5 ){//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){//goal object detected
				timer->Start();
				timer->Reset();
				shooterState = 89;
			}
			break;
		}
	}

/*	void advancedShoot(void)
	{
		printf("Shooter State: %d\n", shooterState);
		//printf("gamepad dY: %d\n", m_Gamepad->GetPOV(0));
		switch(shooterState)
		{
		case 0:
			//everything is off
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(!m_shooterHomeSwitch->Get())
				shooterState = 10;
			else

				shooterState = 20;
			break;
		case 10:
			//cylinder = extend|angle = home
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			autoShooter(HOME_SHOOTER);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			else if(m_Gamepad->GetRawButton(GP_B))
				shooterState = 60;
			else if(m_Gamepad->GetRawButton(GP_Y))
				shooterState = 11;
			break;
		case 11:
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			autoShooter(HOME_SHOOTER);
			m_shootE->Set(false);
			m_shootR->Set(true);
			if(!m_Gamepad->GetRawButton(GP_Y))
				shooterState = 50;
			break;
		case 20:
			//cylinder = extend|angle = pickup
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			autoShooter(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			else if(m_Gamepad->GetRawButton(GP_R))
				shooterState = 30;
			break;
		case 30:
			//cylinder = extend|angle = pickup|shooter = in
			shooter1->Set(SPEED_RPM/2.f);
			shooter2->Set(-SPEED_RPM/2.f);
			autoShooter(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			else if(m_Gamepad->GetRawButton(GP_L))
				shooterState = 20;
			else if(m_intakeHomeSwitch->Get() == CLOSED)
				shooterState = 40;
			break;
		case 39:
			//cylinder = extend | angle = pickup | shooter out
			shooter1->Set(SPEED_RPM/2.f);
			shooter2->Set(SPEED_RPM/2.f);
			m_shooter->Set(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_intakeHomeSwitch->Get() == OPEN)
				shooterState = 20;
			break;
		case 40:
			//cylinder = extend|angle = pickup
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			autoShooter(PICKUP);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetRawButton(GP_L))
				shooterState = 20;
			else if(m_intakeHomeSwitch->Get() == OPEN)
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
			autoShooter(HOME_SHOOTER);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			//cylinder = extend|angle = home
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 40;
			else if(m_Gamepad->GetRawButton(GP_B))
				shooterState = 60;
			else if(m_Gamepad->GetRawButton(GP_Y))
				shooterState = 11;
			break;
		case 60:
			m_shootE->Set(true);
			m_shootR->Set(false);
			autoShooter(SHOOT_FAR);
			shooter1->Set(0.f);
			shooter2->Set(0.f);
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
			shooter1->Set(SPEED_RPM/4.f);
			shooter2->Set(-SPEED_RPM/4.f);
			autoShooter(SHOOT_FAR);
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
			shooter1->Set(SPEED_RPM/4.f);
			shooter2->Set(-SPEED_RPM/4.f);
			autoShooter(SHOOT_FAR);
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
			autoShooter(SHOOT_FAR);
			//shooter1->SetSetpoint(-SHOOT_SPEED);
			//shooter2->SetSetpoint(SHOOT_SPEED);
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
			printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());
			//cylinder = retract|shooter = out|angle = shoot
			if(m_Gamepad->GetRawButton(GP_B))
			{
				timer->Start();
				shooterState = 69;
			}
			else if(m_Gamepad->GetRawButton(GP_X) )//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
			{
				timer->Start();
				shooterState = 80;
			}
			else if(m_Gamepad->GetRawButton(GP_A))
				shooterState = 90;
			break;
		case 80:
			printf("Shooting...");
			// shooter1->SetSetpoint(-SHOOT_SPEED);
			// shooter2->SetSetpoint(SHOOT_SPEED);
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
			autoShooter(SHOOT_FAR);
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

		case 81:
			printf("Shooting...");
			// shooter1->SetSetpoint(-SHOOT_SPEED);
			// shooter2->SetSetpoint(SHOOT_SPEED);
			shooter1->Set(-SPEED_RPM);
			shooter2->Set(SPEED_RPM);
			autoShooter(findShooterAngle());
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
			autoShooter(findShooterAngle());
			if(!m_Gamepad->GetRawButton(GP_A))
				shooterState = 70;
			if(aimAtTarget() == 1 ){//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){//goal object detected
				timer->Start();
				shooterState = 81;
			}
			break;
		}
	}*/


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

		printf("shooter rotate power: %f \t error: %d \t target: %d \n", rotate, currentTicks - ticks, ticks);

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

	void manualServo(void){
		if(m_Joystick->GetRawButton(2))
			m_shooterServo->SetAngle(SERVO_OUT);
		else
			m_shooterServo->SetAngle(SERVO_IN);
	}

	//===============================================VISION FUNCTIONS=============================================

	int aim_loop_counter;
	float aim_fly_trim;
#define HFOV 0.449422282f
#define PI 3.14159265f
#define AUTO_AIM_CORRECTION -1
	float getAutoAimAngle(){
		int image_error;
		image_error = FindTargetCenter();
		if (image_error == 0){
			float angle = atan((centerx - (RES_X/2))*tan(HFOV)/(RES_X/2))*180/PI - atan(AIM_CORRECTION*tan(HFOV)/(RES_X/2))*180/PI;
			printf("\t\tCalced angle from bot %f\n", angle);
			return nav->GetYaw() + angle;
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
			}

			//find filtered blobs
			imaqParticleFilter4(particle, processed, filterCriteria, CRITERIA_COUNT, &filterOptions, NULL, &num_particlesFound);


			if (num_particlesFound == 0){
				//unable to find target
				DriverStation::ReportError("ERROR! Target not found!\n");
				imaqDrawShapeOnImage(frame, frame, {0, IMAGE_CENTER+AIM_CORRECTION, RES_Y-1, 1}, IMAQ_DRAW_VALUE, IMAQ_SHAPE_RECT, 0.0f);
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
