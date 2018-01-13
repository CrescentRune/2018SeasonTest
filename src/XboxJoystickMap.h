/*
 * XboxJoystickMap.h
 *
 *  Created on: Jan 18, 2016
 *      Author: Garrett Davidson
 */

#ifndef SRC_XBOXJOYSTICKMAP_H_
#define SRC_XBOXJOYSTICKMAP_H_

enum XboxJoystickButton {
	XboxButtonA = 1,
	XboxButtonB,
	XboxButtonX,
	XboxButtonY,
	XboxButtonLeftBumper,
	XboxButtonRightBumper,
	XboxButtonBack,
	XboxButtonStart,
	XboxButtonLeftStick,
	XboxButtonRightStick
};

enum XboxJoystickAxis {
	XboxAxisLeftStickX = 0,
	XboxAxisLeftStickY,
	XboxAxisLeftTrigger,
	XboxAxisRightTrigger,
	XboxAxisRightStickX,
	XboxAxisRightStickY
};

enum XboxDPadDirection {
	XboxDPadUp = 0,
	XboxDPadUpLeft = 45,
	XboxDPadLeft = 90,
	XboxDPadDownLeft = 135,
	XboxDPadDown = 180,
	XboxDPadDownRight = 225,
	XboxDPadRight = 270,
	XboxDPadUpRight = 315
};

#endif /* SRC_XBOXJOYSTICKMAP_H_ */
