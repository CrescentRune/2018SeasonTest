/*
 * RowdyJr.h
 *
 *  Created on: Jan 10, 2018
 *      Author: WBI
 */

#ifndef SRC_ROWDYJR_H_
#define SRC_ROWDYJR_H_

enum EncoderPorts {
	EncoderLeftA = 0, EncoderLeftB, EncoderRightA = 4, EncoderRightB
};

enum PWM {
	DriveBackLeft = 0, DriveFrontLeft, DriveBackRight, DriveFrontRight
};

enum FrontSwitchState {
	dataError = 0, LeftSwitch, RightSwitch
};

enum AutoStates {
	InitialStart = 0,
	TurnDownMiddle,
	DriveDiagonal,
	FaceSwitch,
	DriveSideSwitch,
	DeployBlock
};
#endif /* SRC_ROWDYJR_H_ */
