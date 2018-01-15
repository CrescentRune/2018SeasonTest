/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.          													*/
/* Author: Garrett                                                      						*/
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include "WPILib.h"
#include <Encoder.h>
#include "ctre/Phoenix.h"


#include "RowdyJr.h"
#include "THRSTMSTRmap.h"

class Robot : public frc::IterativeRobot {
public:
	DifferentialDrive *driveTrain;
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	TalonSRX *talon;
	VictorSPX *victor;
	PowerDistributionPanel *pdp;
	Encoder *leftDriveEncoder;
	Encoder *rightDriveEncoder;
	ADXRS450_Gyro *gyroscope;
	int ourSwitch, autoState, initialDist, lTurn1, lTurn2, lDrive3, lDrive2, rTurn1, rTurn2, rDrive3, rDrive2;
	double autoDriveSpeed, autoTurnSpeed;

	void RobotInit() {
		//driveTrain = new RobotDrive(DriveFrontLeft, DriveBackLeft, DriveFrontRight, DriveBackRight);
		Talon *frontLeft = new Talon(DriveFrontLeft);
		Talon *backLeft = new Talon(DriveBackLeft);
		Talon *frontRight = new Talon(DriveFrontRight);
		Talon *backRight = new Talon(DriveBackRight);
		gyroscope = new ADXRS450_Gyro();
		talon = new TalonSRX(1);
		victor = new VictorSPX(2);
//		talon->Follow(*victor);
		pdp = new PowerDistributionPanel(0);
		leftDriveEncoder = new Encoder(EncoderLeftA, EncoderLeftB);
		rightDriveEncoder = new Encoder(EncoderRightA, EncoderRightB);

		SpeedControllerGroup *left = new SpeedControllerGroup(*frontLeft,
				*backLeft);
		SpeedControllerGroup *right = new SpeedControllerGroup(*frontRight,
				*backRight);
		driveTrain = new DifferentialDrive(*left, *right);
		leftJoystick = new Joystick(0);
		rightJoystick = new Joystick(1);
		CameraServer::GetInstance()->AddAxisCamera("axis-camera.local");
		ourSwitch = 0;
		AutoShuffleboardPost();
		ShuffleboardPost();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		ShuffleboardPost();
		std::string gameData;
		autoState = InitialStart;
		EncoderReset();
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData[0] == 'L') {
			ourSwitch = LeftSwitch;
		} else {
			ourSwitch = RightSwitch;
		}
	}

	void AutonomousPeriodic() {
		double leftEncDist = leftDriveEncoder->GetDistance();
		double gyroAngle = gyroscope->GetAngle();
		AutoShuffleboardGet();
		SmartDashboard::PutNumber("AutoCase", autoState);

		switch (autoState) {
		case (InitialStart):
			if (leftEncDist > initialDist) {
				driveTrain->TankDrive(autoDriveSpeed, autoDriveSpeed);
				break;
			} else {
				autoState = TurnDownMiddle;
				gyroscope->Reset();
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
				gyroscope->Reset();
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
				gyroscope->Reset();
				autoState = DeployBlock;
				break;
			}
		case (DeployBlock):
			talon->Set(ControlMode::PercentOutput, 1.0);
		}
	}

	void EncoderReset() {
		leftDriveEncoder->Reset();
		rightDriveEncoder->Reset();
	}

	void TeleopInit() {
		EncoderReset();
		ShuffleboardPost();
		gyroscope->Reset();
	}

	void TeleopPeriodic() {
		ShuffleboardPost();
		double left = -leftJoystick->GetRawAxis(yAxisJS);
		double right = -rightJoystick->GetRawAxis(yAxisJS);
		driveTrain->ArcadeDrive(left * 0.65, right * 0.65);

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

	void AutoShuffleboardPost() {
		SmartDashboard::PutNumber("Gyro", gyroscope->GetAngle());
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
	}


	void AutoShuffleboardGet() {
		initialDist    = SmartDashboard::GetNumber("Auton/initDist", -420);
		lTurn1         = SmartDashboard::GetNumber("Auton/lTurn1", -35);
		lTurn2         = SmartDashboard::GetNumber("Auton/lTurn2", 45);
		lDrive2        = SmartDashboard::GetNumber("Auton/lDrive2", -1530);
		lDrive3        = SmartDashboard::GetNumber("Auton/lDrive3", -500);
		rTurn1         = SmartDashboard::GetNumber("Auton/rTurn1", 55);
		rTurn2         = SmartDashboard::GetNumber("Auton/rTurn2", -45);
		rDrive2        = SmartDashboard::GetNumber("Auton/rDrive2", -1530);
		rDrive3        = SmartDashboard::GetNumber("Auton/rDrive3", -800);
		autoDriveSpeed = SmartDashboard::GetNumber("Auton/autoDriveSpeed", 0.8);
		autoTurnSpeed  = SmartDashboard::GetNumber("Auton/autoTurnSpeed", 0.65);
	}

	void ShuffleboardPost() {
		SmartDashboard::PutData("Pdp", pdp);
		SmartDashboard::PutNumber("Talon", talon->GetSelectedSensorPosition(0));
		SmartDashboard::PutData("Drive", driveTrain);
		SmartDashboard::PutData("Pdp", pdp);
		SmartDashboard::PutNumber("Gyro", gyroscope->GetAngle());
		SmartDashboard::PutData("Left DriveEncoder", leftDriveEncoder);
		SmartDashboard::PutNumber("Right DriveEncoder",
				leftDriveEncoder->GetDistance());
		SmartDashboard::GetNumber("Autonomous speed: ", 0.75);
	}

	void DisabledPeriodic(){
		SmartDashboard::PutNumber("Auton/TestValue", 14);

	}

	void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)
