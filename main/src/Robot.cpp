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
#define PICKUP 1800
#define SHOOT_FAR 425
#define SHOOT_CLOSE 550
#define HOME 0
#define SPEED_RPM 1500
#define SHOOTER_SPEED_CHECK 20000

//#define PRACTICE_BOT

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();

	int autoState, autoMode, autoPosition, shooterState;
#ifdef PRACTICE_BOT
	Victor *m_leftDrive4; //0
	Victor *m_leftDrive1; //1
	Victor *m_rightDrive2; //2
	Victor *m_rightDrive3; //3
#else
	VictorSP *m_leftDrive4; //0
	VictorSP *m_leftDrive1; //1
	VictorSP *m_rightDrive2; //2
	VictorSP *m_rightDrive3; //3
#endif
	float leftSpeed, rightSpeed;
	CANTalon *shooter1, *shooter2, *m_shooter;
	Relay *m_LED;
#ifdef PRACTICE_BOT
	CANTalon *m_pusher;
#else
	CANTalon *m_intake;
	VictorSP *m_pusher;
#endif
	Solenoid *m_shiftHigh, *m_shiftLow;
	Solenoid *m_shootE, *m_shootR;

	SimPID *drivePID;
	SimPID *turnPID;
	SimPID *visionPID;
	SimPID *armPID;

	Joystick *m_Gamepad;
	Joystick *m_Joystick;

	VictorSP *m_intakeRoller;

	Timer *timer;
	double last_time;

	DigitalInput *m_shooterHomeSwitch;
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
#define CRITERIA_COUNT 4
	ParticleFilterCriteria2 filterCriteria[CRITERIA_COUNT];
	int num_particlesFound;
	MeasurementType measurements[1];

//====================================================INIT==============================================
	void RobotInit(void) override
	{
#ifdef PRACTICE_BOT
		m_leftDrive4 = new Victor(4);
		m_leftDrive1 = new Victor(1);
		m_rightDrive2 = new Victor(2);
		m_rightDrive3 = new Victor(3);
#else
		m_leftDrive4 = new VictorSP(4);
		m_leftDrive1 = new VictorSP(1);
		m_rightDrive2 = new VictorSP(2);
		m_rightDrive3 = new VictorSP(3);
#endif

		//talon = new CANTalon(1);
		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
		m_shootE = new Solenoid(3);
		m_shootR = new Solenoid(2);

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
		m_intake->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		m_intake->SetControlMode(CANTalon::kPosition);
		m_intake->SetCloseLoopRampRate(0);
		m_intake->ConfigEncoderCodesPerRev(4096);
		m_intake->SetPID(0, 0, 0, 0);
		m_intake->SelectProfileSlot(0);
		m_intake->SetAllowableClosedLoopErr(1000);

		m_intakeRoller = new VictorSP(6);

		m_LED = new Relay(0);

		drivePID = new SimPID(0.0005 ,0, 0, 100);
		drivePID->setMinDoneCycles(1);
		drivePID->setMaxOutput(0.7);
		turnPID = new SimPID(0.1, 0, 0, 2);
		turnPID->setMinDoneCycles(1);
		turnPID->setMaxOutput(0.3);

		visionPID = new SimPID(0.006, 0.02, 0.002, 5);
		visionPID->setMaxOutput(0.3);
		visionPID->setMinDoneCycles(10);

		armPID = new SimPID(0.004, 0, 0.001, 10);
		armPID->setMaxOutput(0.4);

		m_shooterHomeSwitch = new DigitalInput(1);
		m_boulderSwitch = new DigitalInput(0);

		m_leftDriveEncoder = new Encoder(4, 5);
		m_rightDriveEncoder = new Encoder(2, 3);

		m_shooter = new CANTalon(2);
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
			shooter1->SetPosition(0);
			shooter2->SetPosition(0);
		}
	}

	//========================================================AUTONOMOUS=======================================
	void AutonomousInit(void)
		{
			autoState = 0;
			//nav->Reset();
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

			case 1: //drive forward with boulder preloaded and move under low bar
				switch(autoState)
				{
				case 0:
					m_shooter->SetPosition(HOME);
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
#define AUTO_OVER_MOAT -15250
#define AUTO_OVER_OTHER -13000
#define AUTO_LOWBAR_DRIVE -16000
#define AUTO_AIM_POS_1 50
#define AUTO_AIM_POS_2 30
#define AUTO_AIM_POS_3 10
#define AUTO_AIM_POS_4 -5
#define AUTO_AIM_POS_5 -20

			case 4: // drive over flat defense in any position and shoot high goal
				switch(autoState){
				case 0:
					autoArm(HOME);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					autoState++;
					break;
				case 1: //drive over defense
					autoArm(HOME);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					if (autoDrive(AUTO_OVER_OTHER, 0)){
						autoState++;
					}
					break;
				case 2: //aim at tower roughly, prep ball
					autoArm(SHOOT_FAR);
					shooter1->Set(SPEED_RPM/4.f);
					shooter2->Set(-SPEED_RPM/4.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					int result;
					switch(autoPosition){
					case 2:
						result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_2);
						break;
					case 3:
						result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_3);
						break;
					case 4:
						result = autoDrive(AUTO_OVER_OTHER, AUTO_AIM_POS_4);
						break;
					case 5:
						result = autoDrive(AUTO_OVER_MOAT, AUTO_AIM_POS_5);
						break;
					}
					if(result)
					{
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 3: //confirm aim with vision
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM);
					autoArm(SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(aimAtTarget() == 1 && timer->Get() > 0.5){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 4: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM);
					autoArm(SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//cylinder = extend|shooter = out|angle = shoot
					if(timer->Get() > 0.5)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 5: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoArm(HOME);
					m_shootE->Set(false);
					m_shootR->Set(true);
					break;
				}
				break;

			case 5: //under lowbar, shoot with vision high goal
				switch(autoState)
				{
				case 0:
					autoArm(HOME);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					drivePID->setMaxOutput(0.4);
					autoState++;
					break;
				case 1: //drive under lowbar
					autoArm(PICKUP);
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					m_shootE->Set(true);
					m_shootR->Set(false);
					if(autoDrive(AUTO_LOWBAR_DRIVE, 0))
						autoState++;
					break;
				case 2: //rough turn, prep shooter
					autoArm(SHOOT_FAR);
					shooter1->Set(SPEED_RPM/4.f);
					shooter2->Set(-SPEED_RPM/4.f);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(autoDrive(AUTO_LOWBAR_DRIVE, AUTO_AIM_POS_1))
						autoState++;
					break;
				case 3: //confirm aim with vision
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM);
					autoArm(SHOOT_FAR);
					m_shootE->Set(false);
					m_shootR->Set(true);
					if(aimAtTarget() == 1 && timer->Get() > 0.5){ //&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){
						autoState++;
						timer->Reset();
						timer->Start();
					}
					break;
				case 4: //shoot the ball
					shooter1->Set(-SPEED_RPM);
					shooter2->Set(SPEED_RPM);
					autoArm(SHOOT_FAR);
					m_shootE->Set(true);
					m_shootR->Set(false);
					//cylinder = extend|shooter = out|angle = shoot
					if(timer->Get() > 0.5)
					{
						autoState++;
						timer->Stop();
						timer->Reset();
					}
					break;
				case 5: //turn off shooter
					shooter1->Set(0.f);
					shooter2->Set(0.f);
					autoArm(HOME);
					m_shootE->Set(false);
					m_shootR->Set(true);
					break;
				}
				break;

			case 10: //from batter, align to high goal and shoot
				switch(autoState)
				{
				case 0:
					m_shooter->SetPosition(HOME);
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
		//simpleShoot();
		advancedShoot();
		pusher();
		simpleIntake();
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
			m_shooter->Set(PICKUP);
		}
		else if (m_Joystick->GetRawButton(8)){
			m_shooter->Set(0);
		}
		else if(m_Joystick->GetRawButton(9))
			m_shooter->Set(SHOOT_FAR);
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
#ifdef PRACTICE_BOT
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
			autoArm(HOME);
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
			autoArm(HOME);
			m_shootE->Set(false);
			m_shootR->Set(true);
			if(!m_Gamepad->GetRawButton(GP_Y))
				shooterState = 50;
			break;
		case 20:
			//cylinder = extend|angle = pickup
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			autoArm(HOME);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;
			else if(m_Gamepad->GetRawButton(GP_R))
				shooterState = 30;
			break;
		case 30:
			//cylinder = extend|angle = pickup|shooter = in
			shooter1->Set(-SPEED_RPM/2.f);
			shooter2->Set(-SPEED_RPM/2.f);
			autoArm(HOME);
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
			shooter1->Set(SPEED_RPM/2.f);
			shooter2->Set(SPEED_RPM/2.f);
			m_shooter->Set(HOME);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_boulderSwitch->Get() == OPEN)
				shooterState = 20;
			break;
		case 40:
			//cylinder = extend|angle = pickup
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			autoArm(PICKUP);
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
			autoArm(HOME);
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
			autoArm(SHOOT_FAR);
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
			shooter1->Set(-SPEED_RPM/4.f);
			shooter2->Set(-SPEED_RPM/4.f);
			autoArm(SHOOT_FAR);
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
			shooter1->Set(-SPEED_RPM/4.f);
			shooter2->Set(-SPEED_RPM/4.f);
			autoArm(SHOOT_FAR);
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
			autoArm(SHOOT_FAR);
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
			autoArm(SHOOT_FAR);
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
			if(aimAtTarget() == 1 ){//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){//goal object detected
				timer->Start();
				shooterState = 80;
			}
			break;
		}
	}
#else
	inline void pusher(void){
		if (m_Joystick->GetRawButton(6))
			m_pusher->SetSpeed(PUSHER_SPEED);
		else if (m_Joystick->GetRawButton(4))
			m_pusher->SetSpeed(-PUSHER_SPEED);
		else
			m_pusher->SetSpeed(0.f);
	}

	void advancedShoot(void)
	{
		//printf("Shooter State: %d\n", shooterState);
		//printf("gamepad dY: %d\n", m_Gamepad->GetPOV(0));
		switch(shooterState)
		{
		case 0:
			//everything is off
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_intakeRoller->SetSpeed(0.f);
			autoArm(HOME);
			autoIntake(HOME);
			m_shootE->Set(true);
			m_shootR->Set(false);
			shooterState = 10;
			break;
		case 10:
			//cylinder = extend|angle = home
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_intakeRoller->SetSpeed(0.f);
			autoArm(HOME);
			autoIntake(HOME);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_UP)
				shooterState = 20;
			break;
		case 20:
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_intakeRoller->SetSpeed(0.f);
			autoArm(HOME);
			autoIntake(54321);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_Gamepad->GetPOV() == GP_DOWN)
				shooterState = 10;			else if(m_Gamepad->GetRawButton(GP_R))
					shooterState = 30;
			break;
		case 30:
			//cylinder = extend|angle = pickup|shooter = in
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			m_intakeRoller->SetSpeed(0.6);
			autoArm(HOME);
			autoIntake(HOME);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_boulderSwitch->Get() == CLOSED)
				shooterState = 40;
			break;
		case 39:
			//cylinder = extend | angle = pickup | shooter out
			shooter1->Set(SPEED_RPM/2.f);
			shooter2->Set(SPEED_RPM/2.f);
			m_intakeRoller->SetSpeed(0.6);
			m_shooter->Set(HOME);
			m_shootE->Set(true);
			m_shootR->Set(false);
			if(m_boulderSwitch->Get() == OPEN)
				shooterState = 20;
			break;
		case 40:
			//cylinder = extend|angle = pickup
			shooter1->Set(0.f);
			shooter2->Set(0.f);
			autoArm(PICKUP);
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
			autoArm(HOME);
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
			autoArm(SHOOT_FAR);
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
			shooter1->Set(-SPEED_RPM/4.f);
			shooter2->Set(-SPEED_RPM/4.f);
			autoArm(SHOOT_FAR);
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
			shooter1->Set(-SPEED_RPM/4.f);
			shooter2->Set(-SPEED_RPM/4.f);
			autoArm(SHOOT_FAR);
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
			autoArm(SHOOT_FAR);
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
			autoArm(SHOOT_FAR);
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
			if(aimAtTarget() == 1 ){//&& (shooter1->GetEncVel() < -SHOOTER_SPEED_CHECK) && (shooter2->GetEncVel() > SHOOTER_SPEED_CHECK)){//goal object detected
				timer->Start();
				shooterState = 80;
			}
			break;
		}
	}
#endif

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

	bool autoArm(int ticks)
	{
		int currentTicks = m_shooter->GetPosition();

		armPID->setDesiredValue(ticks);

		float rotate = armPID->calcPID(currentTicks);

		m_shooter->Set(-rotate);

		//printf("rotate power: %f \t error: %d \t target: %d \n", rotate, currentTicks - ticks, ticks);

		return armPID->isDone();
	}

	bool autoIntake(int ticks)
		{
			int currentTicks = m_shooter->GetPosition();

			armPID->setDesiredValue(ticks);

			float rotate = armPID->calcPID(currentTicks);

			m_intake->Set(-rotate);

			//printf("rotate power: %f \t error: %d \t target: %d \n", rotate, currentTicks - ticks, ticks);

			return armPID->isDone();
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

	//===============================================VISION FUNCTIONS=============================================
#define AIM_CORRECTION 80
#define AIM_FILTER 1
#define AIM_LOOP_WAIT 5
#define AIM_TIMEOUT 2
#define AIM_FINE_LIMIT 20
#define CLOSE_LIMIT 220

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

		if (center_mass_y < CLOSE_LIMIT)
			autoArm(SHOOT_CLOSE);
		else
			autoArm(SHOOT_FAR);

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
			return 0;
		}
		// unable to take picture, return error
		DriverStation::ReportError("ERROR! Unable to take picture!\n");
		return -1;
	}

	void showBlobMeasurements(void){
		double width, left, area, perimeter, height;
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_BOUNDING_RECT_WIDTH, &width);
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_BOUNDING_RECT_LEFT, &left);
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_AREA, &area);
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_PERIMETER, &perimeter);
		imaqMeasureParticle(particle, 0, FALSE, IMAQ_MT_BOUNDING_RECT_HEIGHT, &height);

		printf("width: %d\tleft: %d\tarea: %d\tperimeter: %d\theight: %d\n", (int)width, (int)left, (int)area, (int)perimeter, (int)height);
	}

#define R_THRESHOLD 210
#define G_THRESHOLD 190
#define B_THRESHOLD 210
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
