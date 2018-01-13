/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
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
	int ourSwitch, autoState;
	double autoSpeed = 0.75;

	void RobotInit() {
		//driveTrain = new RobotDrive(DriveFrontLeft, DriveBackLeft, DriveFrontRight, DriveBackRight);
		Talon *frontLeft = new Talon(DriveFrontLeft);
		Talon *backLeft = new Talon(DriveBackLeft);
		Talon *frontRight = new Talon(DriveFrontRight);
		Talon *backRight = new Talon(DriveBackRight);
		gyroscope = new ADXRS450_Gyro();
		talon = new TalonSRX(1);
		victor = new VictorSPX(2);
		talon->Follow(*victor);
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
		SwitchboardPost();
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
		SwitchboardPost();
		std::string gameData;
		autoState = 0;
		EncoderReset();
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData[0] == 'L') {
			ourSwitch = LeftSwitch;
		} else {
			ourSwitch = RightSwitch;
		}

		SmartDashboard::PutNumber()
	}

	void AutonomousPeriodic() {
		double leftEncDist = leftDriveEncoder->GetDistance(), rightEncDist =
				rightDriveEncoder->GetDistance();
		double gyroAngle = gyroscope->GetAngle();

		switch (autoState) {
		case (InitialStart):
			if (leftEncDist > -420) {
				driveTrain->TankDrive(autoDriveSpeed, autoDriveSpeed);
				break;
			} else {
				autoState = TurnDownMiddle;
				gyroscope->Reset();
				break;
			}
		case (TurnDownMiddle):
			if (ourSwitch == LeftSwitch && gyroAngle > -35) {
				driveTrain->TankDrive(-0.65, 0.65);
				break;
			} else if (ourSwitch == RightSwitch && gyroAngle < 55) {
				driveTrain->TankDrive(0.65, -0.65);
				break;
			} else {
				EncoderReset();
				autoState = DriveDiagonal;
				break;
			}
		case (DriveDiagonal):
			if (leftEncDist > -1530) {
				driveTrain->TankDrive(autoSpeed, autoSpeed);
				break;
			} else {
				gyroscope->Reset();
				autoState = FaceSwitch;
				break;
			}
		case (FaceSwitch):
			if (ourSwitch == LeftSwitch && gyroAngle < 45) {
				driveTrain->TankDrive(0.65, -0.65);
				break;
			} else if (ourSwitch == RightSwitch && gyroAngle > -45) {
				driveTrain->TankDrive(-0.65, 0.65);
				break;
			} else {
				EncoderReset();
				autoState = DriveSideSwitch;
				break;
			}
		case (DriveSideSwitch):
			if (leftEncDist > -800) {
				driveTrain->TankDrive(autoSpeed, autoSpeed);
				break;
			} else {
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
		SwitchboardPost();
		gyroscope->Reset();
	}

	void TeleopPeriodic() {
		SwitchboardPost();
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

//		if (leftJoystick->GetRawButton(trigger)) {
//			victor->Set(ControlMode::PercentOutput, 1);
//		} else if (leftJoystick->GetRawButton(leftButton)) {
//			victor->Set(ControlMode::PercentOutput, -1);
//		} else {
//			victor->Set(ControlMode::PercentOutput, 0);
//		}


	}

	void AutoSwitchboardPost() {
		SmartDashboard::PutNumber("Gyro", gyroscope->GetAngle());
		SmartDashboard::PutData("Left DriveEncoder", leftDriveEncoder);
		SmartDashboard::PutData("Right DriveEncoder", leftDriveEncoder);
	}

	void SwitchboardPost() {
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

	void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)
