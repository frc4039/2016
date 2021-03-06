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

#define SHOOT_SPEED 0.8225f
#define PICKUP 1720
#define SHOOT_LOWBAR 370
#define SHOOT_FAR 390
#define SHOOT_CLOSE 540
#define HOME_SHOOTER 0
#define HOME_INTAKE 0
#define TRANSFER 0
#define INTAKE_SHOOT_FAR 475
#define INTAKE_SHOOT_CLOSE 475
#define SPEED_RPM 1500
#define SHOOTER_SPEED_CHECK 20000

#define SERVO_IN 90
#define SERVO_OUT 40

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();

	int autoState, autoMode, autoPosition, shooterState;
	VictorSP *m_leftDrive4; //0
	VictorSP *m_leftDrive1; //1
	VictorSP *m_rightDrive2; //2
	VictorSP *m_rightDrive3; //3

	VictorSP *m_pusher;
	VictorSP *m_intakeRoller;

	float leftSpeed, rightSpeed;

	CANTalon *shooter1, *shooter2, *m_shooter, *m_intake;

	Relay *m_LED;
	Servo *m_shooterServo;

	Solenoid *m_shiftHigh, *m_shiftLow;
	Solenoid *m_shootE, *m_shootR;

	SimPID *drivePID;
	SimPID *turnPID;
	SimPID *visionPID;
	SimPID *shooterPID;
	SimPID *intakePID;

	Joystick *m_Gamepad;
	Joystick *m_Joystick;



	Timer *timer;
	double last_time;

	DigitalInput *m_shooterHomeSwitch;
	DigitalInput *m_pusherHomeSwitch;
	DigitalInput *m_intakeHomeSwitch;
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
	double rect_left, rect_width, target_y, centerx, last_turn;


	//vision filter options
	ParticleFilterOptions2 filterOptions;
#define CRITERIA_COUNT 4
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

		//talon = new CANTalon(1);
		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
		m_shootE = new Solenoid(2);
		m_shootR = new Solenoid(3);

		m_Joystick = new Joystick(0);
		m_Gamepad = new Joystick(1);

		shooter1 = new CANTalon(0);
		shooter1->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		shooter1->SetControlMode(CANTalon::kSpeed);
		shooter1->SetCloseLoopRampRate(0);
		shooter1->ConfigEncoderCodesPerRev(4096);
		shooter1->SetPID(0.1, 0, 0.01, 0);
		shooter1->SelectProfileSlot(0);
		// shooter1->SetClosedLoopOutputDirection(true);
		// shooter1->SetSensorDirection(true);
		shooter1->SetAllowableClosedLoopErr(1000);

		shooter2 = new CANTalon(1);
		shooter2->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		shooter2->SetControlMode(CANTalon::kSpeed);
		shooter2->SetCloseLoopRampRate(0);
		shooter2->ConfigEncoderCodesPerRev(4096);
		shooter2->SetPID(0.1, 0, 0.01, 0);
		shooter2->SelectProfileSlot(0);
		// shooter2->SetSensorDirection(false);
		shooter2->SetAllowableClosedLoopErr(1000);

		m_intake = new CANTalon(4);
		/*m_intake->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		m_intake->SetControlMode(CANTalon::kPosition);
		m_intake->SetCloseLoopRampRate(0);
		m_intake->ConfigEncoderCodesPerRev(4096);
		m_intake->SetPID(0, 0, 0, 0);
		m_intake->SelectProfileSlot(0);
		m_intake->SetAllowableClosedLoopErr(1000);
*/
		m_intakeRoller = new VictorSP(6);

		m_LED = new Relay(0);

		drivePID = new SimPID(0.0005 ,0, 0, 100);
		drivePID->setMinDoneCycles(1);
		drivePID->setMaxOutput(0.7);
		turnPID = new SimPID(0.1, 0, 0, 2);
		turnPID->setMinDoneCycles(1);
		turnPID->setMaxOutput(0.3);

		visionPID = new SimPID(0.00675, 0.02, 0.001, 5);
		visionPID->setMaxOutput(0.4);
		visionPID->setMinDoneCycles(10);

		shooterPID = new SimPID(0.002, 0, 0.001, 10);
		shooterPID->setMaxOutput(0.4);

		intakePID = new SimPID(0.001, 0, 0.001, 10);
		intakePID->setMaxOutput(0.6);

		m_shooterHomeSwitch = new DigitalInput(0);
		//m_boulderSwitch = new DigitalInput(0);
		m_intakeHomeSwitch = new DigitalInput(1);

		m_leftDriveEncoder = new Encoder(5, 4);
		m_rightDriveEncoder = new Encoder(3, 2);

		m_shooter = new CANTalon(2);
		m_shooter->SetFeedbackDevice(CANTalon::QuadEncoder);
		/*
		m_shooter->SetPID(1,0,0.4,1);
		m_shooter->SetFeedbackDevice(CANTalon::QuadEncoder);
		m_shooter->SetIzone(100);
		m_shooter->SetCloseLoopRampRate(100);
		m_shooter->SelectProfileSlot(0);
		m_shooter->SetControlMode(CANTalon::kPosition);
		m_shooter->SetClosedLoopOutputDirection(true);
*/

		nav = new AHRS(SPI::Port::kMXP);

		m_pusher = new VictorSP(5);
		m_shooterServo = new Servo(7);

		timer = new Timer();
		timer->Reset();

		VisionInit();

		shooterState = 0;
		autoState = 0;
		autoMode = 0;
		autoPosition = 0;
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
		filterCriteria[0].lower = 100000;
		filterCriteria[0].upper = 200000;

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
		//printf("shooter Speed: %f\t%f\n", shooter1->GetSpeed(), shooter2->GetSpeed());

		if(m_shooterHomeSwitch->Get() == CLOSED)
			m_shooter->SetPosition(0);
		if(m_intakeHomeSwitch->Get() == CLOSED)
			m_intake->SetPosition(0);

		for (int i = 1; i <= 12; i++)
		{
			if(m_Joystick->GetRawButton(i))
			{
				if (i < 7)
					autoMode = i;
				else if (i >= 7 && i < 12)
					autoPosition = i - 6;
				nav->Reset();
				m_leftDriveEncoder->Reset();
				m_rightDriveEncoder->Reset();
				DriverStation::ReportError("Auto mode: " + std::to_string((long)autoMode) + " position: " + std::to_string((long)autoPosition) + "\n");
			}
		}
		DriverStation::ReportError("Gyro: " + std::to_string((float)nav->GetYaw()) +
				" enc: " + std::to_string((long)m_leftDriveEncoder->Get()) +
				", " + std::to_string((long)m_rightDriveEncoder->Get()) + "\n");


		if(m_Joystick->GetRawButton(10)){
			m_shooter->SetPosition(0);
			m_intake->SetPosition(0);
			shooter1->SetPosition(0);
			shooter2->SetPosition(0);

		}

		//printf("Shooter angle: %d \n", m_shooter->GetEncPosition());
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
			timer->Start();
			IMAQdxStartAcquisition(session);
			m_LED->Set(Relay::kForward);
		}

		void AutonomousPeriodic(void)
		{
			switch(autoMode)
			{
			case 0:
				m_leftDrive4->SetSpeed(0.f);
				m_leftDrive1->SetSpeed(0.f);
				m_rightDrive2->SetSpeed(0.f);
				m_rightDrive3->SetSpeed(0.f);
				break;

			/*case 1: //drive forward with boulder preloaded and move under low bar
				switch(autoState)
				{
				case 0:
					m_shooter->SetPosition(HOME_SHOOTER);
					autoState++;
					break;
				case 1:
					m_shootE->Set(true);
					m_shootR->Set(false);
					autoDrive(7000, 0);
					break;
				}
				break;
			case 2: //wait 5 secs, drive forward with boulder preloaded and move under low bar
				switch(autoState)
				{
				case 0:
					m_shooter->SetPosition(HOME);
					if(timer->Get() > 5.0)
					{
						timer->Stop();
						timer->Reset();
						autoState++;
					}
					break;
				case 1:
					m_shootE->Set(true);
					m_shootR->Set(false);
					autoDrive(5000, 0);
					break;
				}
				break;
			case 3: //drive forward with boulder preloaded, cross low bar, discharge ball to tower, revere to neutral zone through low bar, ready to get next boulder
				switch(autoState)
				{
				case 0:
					m_shooter->SetPosition(HOME);
					autoState++;
					break;
				case 1:
					m_shootE->Set(true);
					m_shootR->Set(false);
					if(autoDrive(10000, 0))
						autoState++;
					break;
				case 2:
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooter->Set(PICKUP);
					if(autoDrive(10000, 45))
					{
						timer->Reset();
						autoState++;
					}
					break;
				case 3:
					m_shootE->Set(true);
					m_shootR->Set(false);
					shooter1->SetSetpoint(-SHOOT_SPEED/4);
					shooter2->SetSetpoint(SHOOT_SPEED/4);
					if(m_boulderSwitch->Get() == OPEN)
					{
						shooter1->SetSetpoint(0.f);
						shooter2->SetSetpoint(0.f);
						autoState++;
					}
					break;
				case 5:
					m_shootE->Set(true);
					m_shootR->Set(false);
					autoDrive(0, 0);
					break;
				}
				break;
				*/
#define AUTO_OVER_MOAT -15250
#define AUTO_OVER_OTHER -13000
#define AUTO_LOWBAR_DRIVE -16000
#define AUTO_AIM_POS_1 50
#define AUTO_AIM_POS_2 30
#define AUTO_AIM_POS_3 10
#define AUTO_AIM_POS_4 -5
#define AUTO_AIM_POS_5 -20
#define AUTO_SHOOTER_POS SHOOT_FAR+40

			case 4: // drive over flat defense in any position and shoot high goal
				printf("autoState: %d\n", autoState);
				switch(autoState){
				case 0:
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);
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
					m_shooterServo->SetAngle(SERVO_IN);
					if (autoDrive(AUTO_OVER_OTHER, 0)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;

				case 2: //transfer ball, aim at tower roughly
					autoShooter(AUTO_SHOOTER_POS);
					autoIntake(INTAKE_SHOOT_FAR);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(-1);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);
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
						result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_5);
						break;
					}
					if(result && timer->Get() > 1.5)
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 3: //prep ball, confirm aim with vision
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					autoShooter(AUTO_SHOOTER_POS);
					autoIntake(INTAKE_SHOOT_FAR);
					m_intakeRoller->SetSpeed(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);
					if(aimAtTarget() == 1 && timer->Get() > 0.75){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 4: //shoot the ball
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooterServo->SetAngle(SERVO_OUT);
					m_intakeRoller->SetSpeed(0.f);
					if(timer->Get() > 0.75)
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
					m_shooterServo->SetAngle(SERVO_IN);
					m_intakeRoller->SetSpeed(0.f);
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
					m_shooterServo->SetAngle(SERVO_IN);
					drivePID->setMaxOutput(0.5);
					autoState++;
					break;
				case 1: //drive under lowbar
					autoShooter(HOME_SHOOTER);
					autoIntake(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
						{
							autoState++;
							timer->Reset();
							timer->Start();
						}
					break;
				case 2: //move intake to transfer
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooterServo->SetAngle(SERVO_IN);
					if(timer->Get() > 1.5)
						autoState++;
					break;
				case 3: //rough turn, transfer ball
					autoShooter(HOME_SHOOTER);
					autoIntake(TRANSFER);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_intakeRoller->SetSpeed(-0.6);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);
					if(autoDrive(AUTO_LOWBAR_DRIVE, AUTO_AIM_POS_1))
						autoState++;
					break;
				case 4: //confirm aim with vision
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					m_intakeRoller->SetSpeed(0.f);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);
					if(aimAtTarget() == 1 && timer->Get() > 1.0){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 5: //shoot the ball
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					autoShooter(SHOOT_FAR);
					autoIntake(INTAKE_SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooterServo->SetAngle(SERVO_OUT);

					if(timer->Get() > 0.5)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 6: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoShooter(HOME_SHOOTER);
					autoIntake(HOME_INTAKE);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);
					break;
				}
				break;

			case 10: //from batter, align to high goal and shoot
				switch(autoState)
				{
				case 0:
					m_shooter->SetPosition(HOME_SHOOTER);
					autoState++;
					break;
				case 1:
					m_shootE->Set(true);
					m_shootR->Set(false);
					if (autoDrive(2000, 0))
						autoState++;
					break;
				case 2:
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooter->SetPosition(SHOOT_FAR);
					autoState++;
					break;
				case 3:
					m_shootE->Set(false);
				}
				break;
			}
		}

	//===========================================================TELEOP=======================================
	void TeleopInit(void)
	{
		IMAQdxStartAcquisition(session);
		if(m_Joystick->GetRawButton(9))
			m_shooter->SetPosition(0);
		m_LED->Set(Relay::kForward);
	}

	void TeleopPeriodic(void)
	{
		operateShifter();
		//manualServo();
		//simpleShoot();
		advancedShoot();
		pusher();
		//simpleIntake();
		//SubtractionFilter();
		//if (m_pusherHomeSwitch->Get())
			//m_pusher->Reset();
		//if (m_Joystick->GetRawButton(10))
			//aimAtTarget();
		if(shooterState != 90){
			FindTargetCenter();
			teleDrive();
		}
		//tempIntake();


		//lw->Run();
	}

	//==========================================================USER FUNCTIONS=================================
	void advancedShoot(void)
			{
				printf("Shooter State: %d\n", shooterState);
				//printf("gamepad dY: %d\n", m_Gamepad->GetPOV(0));
				if(m_Gamepad->GetRawButton(GP_L) && shooterState != 50)
					m_intakeRoller->SetSpeed(0.6);
				else if(m_Gamepad->GetRawButton(GP_R) && shooterState != 50)
					m_intakeRoller->SetSpeed(-0.6);
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
					m_shooterServo->SetAngle(SERVO_IN);

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
					m_shooterServo->SetAngle(SERVO_IN);

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
					m_shooterServo->SetAngle(SERVO_OUT);

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
					m_shooterServo->SetAngle(SERVO_IN);

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
					m_shooterServo->SetAngle(SERVO_IN);

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
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);

					autoIntake(TRANSFER);
					m_intakeRoller->SetSpeed(-0.6);

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
					m_shooterServo->SetAngle(SERVO_IN);

					autoIntake(INTAKE_SHOOT_FAR);
					//m_intakeRoller->SetSpeed(0.f);

					if(m_Gamepad->GetPOV() == GP_DOWN)
						shooterState = 10;
					if(m_Gamepad->GetPOV() == GP_UP)
						shooterState = 20;
					else if(m_Gamepad->GetRawButton(GP_X) || m_Gamepad->GetRawButton(GP_A))
						{
							shooterState = 70;
							timer->Reset();
							timer->Start();
						}
					break;

				case 69: //cancel shot
					autoShooter(SHOOT_FAR);
					shooter1->Set(-SPEED_RPM/4);
					shooter2->Set(SPEED_RPM/4);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);

					autoIntake(TRANSFER);
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
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);

					autoIntake(INTAKE_SHOOT_FAR);
					//m_intakeRoller->SetSpeed(0.f);

					printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());
					//cylinder = retract|shooter = out|angle = shoot

					if(m_Gamepad->GetRawButton(GP_B))
					{
						timer->Start();
						timer->Reset();
						shooterState = 69;
					}
					else if(m_Gamepad->GetRawButton(GP_X) && timer->Get() > 1.0 )//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
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
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					m_shootE->Set(false);
					m_shootR->Set(true);
					m_shooterServo->SetAngle(SERVO_IN);

					autoIntake(INTAKE_SHOOT_FAR);
					//m_intakeRoller->SetSpeed(0.f);

					printf("shooter Speed: %d\t%d\terror: %d\t%d\n", shooter1->GetEncVel(), shooter2->GetEncVel(), shooter1->GetClosedLoopError(), shooter2->GetClosedLoopError());

					if(m_Gamepad->GetRawButton(GP_B))
					{
						timer->Start();
						shooterState = 10;
					}
					else if(m_Gamepad->GetRawButton(GP_X) && timer->Get() > 2.0 )//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK) )
					{
						timer->Reset();
						timer->Start();
						shooterState = 81;
					}

					break;

				case 80: //shoot from shoot_far position
					printf("Shooting...");
					autoShooter(SHOOT_FAR);
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooterServo->SetAngle(SERVO_OUT);

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
					printf("Shooting...");
					autoShooter(HOME_SHOOTER);
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooterServo->SetAngle(SERVO_OUT);

					autoIntake(INTAKE_SHOOT_FAR);
					//m_intakeRoller->SetSpeed(0.f);

					if(timer->Get() > 1.0)
					{
						shooterState = 10;
						timer->Stop();
						timer->Reset();
					}
					break;

				case 82:
					printf("Shooting...");
					autoShooter(findShooterAngle());
					shooter1->Set(SPEED_RPM);
					shooter2->Set(-SPEED_RPM);
					m_shootE->Set(true);
					m_shootR->Set(false);
					m_shooterServo->SetAngle(SERVO_OUT);

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
					autoShooter(findShooterAngle());
					if(!m_Gamepad->GetRawButton(GP_A))
						shooterState = 70;
					if(aimAtTarget() == 1 && timer->Get() > 0.5 ){//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){//goal object detected
						timer->Start();
						timer->Reset();
						shooterState = 82;
					}
					autoShooter(findShooterAngle());
					break;
				}
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

	bool autoIntake(int ticks)
		{
			int currentTicks = m_intake->GetPosition();

			intakePID->setDesiredValue(ticks);

			float rotate = intakePID->calcPID(currentTicks);

			m_intake->Set(-rotate);

			printf("intake rotate power: %f \t error: %d \t target: %d \n", rotate, currentTicks - ticks, ticks);

			return intakePID->isDone();
		}
	bool autoShooter(int ticks)
	{
		int currentTicks = m_shooter->GetPosition();

		shooterPID->setDesiredValue(ticks);

		float rotate = shooterPID->calcPID(currentTicks);

		m_shooter->Set(-rotate);

		printf("shooter rotate power: %f \t error: %d \t target: %d \n", rotate, currentTicks - ticks, ticks);

		return shooterPID->isDone();
	}



	void manualServo(void){
		if(m_Joystick->GetRawButton(2))
			m_shooterServo->SetAngle(SERVO_OUT);
		else
			m_shooterServo->SetAngle(SERVO_IN);
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



		printf("intake: %d error: %d\n", m_shooter->GetEncPosition(), m_shooter->GetClosedLoopError());
		if(m_Joystick->GetRawButton(10))
			m_shooter->SetPosition(0);

		if(m_Joystick->GetRawButton(7)){
			m_shooter->SetSetpoint(0.6);
		}
		else if (m_Joystick->GetRawButton(8)){
			m_shooter->SetSetpoint(-0.6);
		}
		else
			m_shooter->SetSetpoint(0.f);
		/*
		else{
			m_shooter->SetSetpoint(0.0f);
			m_shooter->SetEncPosition(0);
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
		if (m_Joystick->GetRawButton(4))
			m_pusher->SetSpeed(PUSHER_SPEED);
		else if (m_Joystick->GetRawButton(6))
			m_pusher->SetSpeed(-PUSHER_SPEED);
		else
			m_pusher->SetSpeed(0.f);
	}


	inline void simpleIntake()
	{
		printf("Intake angle: %d \n", m_intake->GetEncPosition());
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



	//===============================================VISION FUNCTIONS=============================================
#define AIM_CORRECTION 45
#define AIM_FILTER 0.50
#define AIM_LOOP_WAIT 2
#define AIM_TIMEOUT 2
#define AIM_FINE_LIMIT 10
#define CLOSE_LIMIT 0

#define IMAGE_CENTER 320
	int aim_loop_counter;

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
			printf("im_error: %d\tcenter: %f\terror: %f\tturn: %f\n", image_error, centerx, error, turn);

			//SmartDashboard::PutNumber("Aim Error", IMAGE_CENTER + AIM_CORRECTION - x_pixel);
			//SmartDashboard::PutNumber("Motor Output", turn);
		}

		if(!IsAutonomous()){
			if (target_y < CLOSE_LIMIT)
				autoShooter(SHOOT_CLOSE);
			else
				autoShooter(SHOOT_FAR);
		}

		//if image hasn't been found in a while then stop moving
		if (aim_loop_counter - AIM_LOOP_WAIT >= AIM_TIMEOUT){
			turn = last_turn = 0;
		}

		aim_loop_counter++;

		humanAdjust = scale(m_Gamepad->GetRawAxis(0), 0.3);
		turn -= humanAdjust;


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

			//find filtered blobs
			imaqParticleFilter4(particle, processed, filterCriteria, CRITERIA_COUNT, &filterOptions, NULL, &num_particlesFound);
			//showBlobMeasurements();
			if(m_Joystick->GetRawButton(12)){
				sprintf(filename, "/home/lvuser/pic%d.bmp", picture_ID);
				DriverStation::ReportError("writing picture to file\n");
				imaqWriteBMPFile(frame, filename, 30, &colourTable);
				picture_ID++;
			}
			else if (num_particlesFound == 0){
				//unable to find target
				DriverStation::ReportError("ERROR! Target not found!\n");
				CameraServer::GetInstance()->SetImage(frame);
				return -3;
			}
			else if (num_particlesFound > 0){
				//take measurements
				int blob = pickBlob(num_particlesFound);
				imaqMeasureParticle(particle, blob, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &rect_left);
				imaqMeasureParticle(particle, blob, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &rect_width);
				imaqMeasureParticle(particle, blob, FALSE, IMAQ_MT_BOUNDING_RECT_TOP, &target_y);


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

	int pickBlob(int num_blobs){
		double width[num_blobs], left[num_blobs], area[num_blobs], perimeter[num_blobs], height[num_blobs], aspect[num_blobs], fill[num_blobs];
		double maxArea = 0, maxAreaIndex = 0;
		for (int i = 0; i < num_particlesFound; i++){
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &width[i]);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &left[i]);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_AREA, &area[i]);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_PERIMETER, &perimeter[i]);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_HEIGHT, &height[i]);

			aspect[i] = height[i] / width[i];
			fill[i] = area[i] / width[i] * height[i];

			//printf("blob%d width: %d\tleft: %d\tarea: %d\tperimeter: %d\theight: %d, aspect: %f, fill: %f\n",
								//i, (int)width[i], (int)left[i], (int)area[i], (int)perimeter[i], (int)height[i], (float)aspect[i], (float)fill[i]);

			if (area[i] > maxArea && area[i] > 1000){
				//printf("area %d is bigger than %d, index %d\n", (int)area[i], (int)maxArea, (int)maxAreaIndex);
				maxArea = area[i];
				maxAreaIndex = i;
			}
		}
		printf("picking blob %d\n", (int)maxAreaIndex);
		return (int)maxAreaIndex;
	}
	void showBlobMeasurements(void){
		double width, left, area, perimeter, height, aspect, fill;
		for (int i = 0; i < num_particlesFound; i++){
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &width);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &left);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_AREA, &area);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_PERIMETER, &perimeter);
			imaqMeasureParticle(particle, i, FALSE, IMAQ_MT_BOUNDING_RECT_HEIGHT, &height);

			aspect = height / width;
			fill = area / (width * height);

			printf("blob%d width: %d\tleft: %d\tarea: %d\tperimeter: %d\theight: %d, aspect: %f, fill: %f\n",
					i, (int)width, (int)left, (int)area, (int)perimeter, (int)height, (float)aspect, (float)fill);
		}
	}

#define R_THRESHOLD 90
#define G_THRESHOLD 100
#define B_THRESHOLD 90
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
	inline int findShooterAngle()
	{
#define SLOPE -0.0779f
#define INTERCEPT 58.699f
#define SHOOTER_TRIM 0.f
		return (SLOPE*target_y + INTERCEPT + SHOOTER_TRIM)*(4096.f/360.f);
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
