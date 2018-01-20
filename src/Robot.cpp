/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.          													*/
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

//Include WPILib and 3rd party libraries
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"

//Include custom headers
#include "RowdyJr.h"
#include "THRSTMSTRmap.h"



class Robot : public frc::IterativeRobot {
public:
	//Controls
	Joystick *leftJoystick;
	Joystick *rightJoystick;

	//Robot parts
	PowerDistributionPanel *pdp;
	DifferentialDrive *driveTrain;
	TalonSRX *talon;
	VictorSPX *victor;

	//Sensors
	Encoder       *leftDriveEncoder;
	Encoder       *rightDriveEncoder;
	ADXRS450_Gyro *gyroscope;
	AHRS          *navxBoard;

	//Custom variables
	int ourSwitch, autoState, initialDist, lTurn1, lTurn2, lDrive3, lDrive2, rTurn1, rTurn2, rDrive3, rDrive2;
	double autoDriveSpeed, autoTurnSpeed;

	void RobotInit() {
		SmartDashboard::PutNumber("Auton/initDist", -420);
		SmartDashboard::PutNumber("Auton/lTurn1", -35);
		SmartDashboard::PutNumber("Auton/lTurn2", 45);
		SmartDashboard::PutNumber("Auton/lDrive2", -1200);
		SmartDashboard::PutNumber("Auton/lDrive3", -550);
		SmartDashboard::PutNumber("Auton/rTurn1", 55);
		SmartDashboard::PutNumber("Auton/rTurn2", -45);
		SmartDashboard::PutNumber("Auton/rDrive2", -1530);
		SmartDashboard::PutNumber("Auton/rDrive3", -600);
		SmartDashboard::PutNumber("Auton/autoDriveSpeed", 0.8);
		SmartDashboard::PutNumber("Auton/autoTurnSpeed", 0.65);
		SmartDashboard::PutNumber("Auton/testnum", 14);

		//Sensor
		leftDriveEncoder  = new Encoder(EncoderLeftA, EncoderLeftB);
		rightDriveEncoder = new Encoder(EncoderRightA, EncoderRightB);
		navxBoard = new AHRS(SerialPort::kMXP);
		CameraServer::GetInstance()->AddAxisCamera("axis-camera.local");


		//Robot parts
		Talon *frontLeft = new Talon(DriveFrontLeft);
		Talon *backLeft = new Talon(DriveBackLeft);
		Talon *frontRight = new Talon(DriveFrontRight);
		Talon *backRight = new Talon(DriveBackRight);
		SpeedControllerGroup *left  = new SpeedControllerGroup(*frontLeft, *backLeft);
		SpeedControllerGroup *right = new SpeedControllerGroup(*frontRight, *backRight);
		driveTrain = new DifferentialDrive(*left, *right);

		//CAN Chain
		pdp = new PowerDistributionPanel(0);
		talon = new TalonSRX(1);
		victor = new VictorSPX(2);
		talon->Follow(*victor);

		//Controller
		leftJoystick  = new Joystick(0);
		rightJoystick = new Joystick(1);

		//Game interaction
		ourSwitch = 0;
		ShuffleboardPost();
	}

	void AutonomousInit() override {
		ShuffleboardPost();
		EncoderReset();

		autoState = InitialStart;

		std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData[0] == 'L') {
			ourSwitch = LeftSwitch;
		} else {
			ourSwitch = RightSwitch;
		}
	}

	void AutonomousPeriodic() {
		double leftEncDist = leftDriveEncoder->GetDistance();
		double gyroAngle = navxBoard->GetYaw();
		AutoShuffleboardGet();

		switch (autoState) {
			case (InitialStart):
				if (leftEncDist > initialDist) {
					driveTrain->TankDrive(autoDriveSpeed, autoDriveSpeed);
					break;
				} else {
					navxBoard->Reset();
					autoState = TurnDownMiddle;
					break;
				}
			case (TurnDownMiddle):
				if (ourSwitch == LeftSwitch && gyroAngle > lTurn1) {
					driveTrain->TankDrive(-autoTurnSpeed, autoTurnSpeed);
					break;
				} else if (ourSwitch == RightSwitch && gyroAngle < rTurn1) {
					driveTrain->TankDrive(autoTurnSpeed, -autoTurnSpeed);
					break;
				} else {
					EncoderReset();
					autoState = DriveDiagonal;
					break;
				}
			case (DriveDiagonal):
				if (ourSwitch == RightSwitch && leftEncDist > rDrive2) {
					driveTrain->TankDrive(autoDriveSpeed, autoDriveSpeed);
					break;
				} else if (ourSwitch == LeftSwitch && leftEncDist > lDrive2) {
					driveTrain->TankDrive(autoDriveSpeed, autoDriveSpeed);
					break;
				}
				else {
					navxBoard->Reset();
					autoState = FaceSwitch;
					break;
				}
			case (FaceSwitch):
				if (ourSwitch == LeftSwitch && gyroAngle < lTurn2) {
					driveTrain->TankDrive(autoTurnSpeed, -autoTurnSpeed);
					break;
				} else if (ourSwitch == RightSwitch && gyroAngle > rTurn2) {
					driveTrain->TankDrive(-autoTurnSpeed, autoTurnSpeed);
					break;
				} else {
					EncoderReset();
					autoState = DriveSideSwitch;
					break;
				}
			case (DriveSideSwitch):
				if (ourSwitch == RightSwitch && leftEncDist > rDrive3) {
					driveTrain->TankDrive(autoDriveSpeed, autoDriveSpeed);
					break;
				} else if ( ourSwitch == LeftSwitch && leftEncDist > lDrive3){
					driveTrain->TankDrive(autoDriveSpeed, autoDriveSpeed);
					break;
				}
				else {
					autoState = DeployBlock;
					break;
				}
			case (DeployBlock):
				talon->Set(ControlMode::PercentOutput, 1.0);
		}
	}

	void EncoderReset(){
		leftDriveEncoder->Reset();
		rightDriveEncoder->Reset();
	}

	void TeleopInit(){
		EncoderReset();
		ShuffleboardPost();
		navxBoard->Reset();
	}

	void TeleopPeriodic() {
		ShuffleboardPost();
		double drive = -rightJoystick->GetRawAxis(yAxisJS);
		double turn = rightJoystick->GetRawAxis(rotateJS);
		driveTrain->ArcadeDrive(drive * 0.65, turn * 0.65);

		if (leftJoystick->GetRawButton(trigger)) {
			talon->Set(ControlMode::PercentOutput, 1);
		} else if (rightJoystick->GetRawButton(trigger)) {
			talon->Set(ControlMode::PercentOutput, -1);
		} else {
			talon->Set(ControlMode::PercentOutput, 0);
		}

//		if (rightJoystick->GetRawButton(trigger)) {
//			victor->Set(ControlMode::PercentOutput, -0.3);
//		} else if (leftJoystick->GetRawButton(leftButton)) {
//			victor->Set(ControlMode::PercentOutput, -1);
//		} else {
//			victor->Set(ControlMode::PercentOutput, 0);
//		}
//		SmartDashboard::PutNumber("Intake speed:", rightJoystick->GetRawAxis(3));

	}

	void AutoShuffleboardGet() {
		initialDist    = SmartDashboard::GetNumber("Auton/initDist", -420);
		lTurn1         = SmartDashboard::GetNumber("Auton/lTurn1", -35);
		lTurn2         = SmartDashboard::GetNumber("Auton/lTurn2", 45);
		lDrive2        = SmartDashboard::GetNumber("Auton/lDrive2", -1530);
		lDrive3        = SmartDashboard::GetNumber("Auton/lDrive3", -500);
		rTurn1         = SmartDashboard::GetNumber("Auton/rTurn1", 55);
		rTurn2         = SmartDashboard::GetNumber("Auton/rTurn2", -45);
		rDrive2        = SmartDashboard::GetNumber("Auton/rDrive2", -1200);
		rDrive3        = SmartDashboard::GetNumber("Auton/rDrive3", -790);
		autoDriveSpeed = SmartDashboard::GetNumber("Auton/autoDriveSpeed", 0.8);
		autoTurnSpeed  = SmartDashboard::GetNumber("Auton/autoTurnSpeed", 0.65);
	}

	void ShuffleboardPost() {
		SmartDashboard::PutData("NavX-Board", navxBoard);
		SmartDashboard::PutNumber("NavX-Yaw", navxBoard->GetYaw());
		SmartDashboard::PutData("Gyro", gyroscope);
		SmartDashboard::PutData("Pdp", pdp);
		SmartDashboard::PutNumber("Talon", talon->GetSelectedSensorPosition(0));
		SmartDashboard::PutData("Drive", driveTrain);
		SmartDashboard::PutData("Pdp", pdp);
		SmartDashboard::PutData("Gyro", gyroscope);
		SmartDashboard::PutData("LeftDriveEncoder", leftDriveEncoder);
		SmartDashboard::PutData("RightDriveEncoder", rightDriveEncoder);
		SmartDashboard::GetNumber("Autonomous speed: ", 0.75);
		SmartDashboard::PutData("NavX-Board", navxBoard);
		SmartDashboard::PutData("Gyro", gyroscope);
		SmartDashboard::PutData("LeftDriveEncoder", leftDriveEncoder);
		SmartDashboard::PutData("RightDriveEncoder", rightDriveEncoder);
		SmartDashboard::PutNumber("Auton/initDist", -420);
		SmartDashboard::PutNumber("Auton/lTurn1", -35);
		SmartDashboard::PutNumber("Auton/lTurn2", 45);
		SmartDashboard::PutNumber("Auton/lDrive2", -1200);
		SmartDashboard::PutNumber("Auton/lDrive3", -550);
		SmartDashboard::PutNumber("Auton/rTurn1", 55);
		SmartDashboard::PutNumber("Auton/rTurn2", -45);
		SmartDashboard::PutNumber("Auton/rDrive2", -1530);
		SmartDashboard::PutNumber("Auton/rDrive3", -600);
		SmartDashboard::PutNumber("Auton/autoDriveSpeed", 0.8);
		SmartDashboard::PutNumber("Auton/autoTurnSpeed", 0.65);
		SmartDashboard::PutNumber("Auton/AutoCase", autoState);
		SmartDashboard::PutNumber("Auton/testnum", 14);
	}

	void DisabledPeriodic(){
	}

	void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)
