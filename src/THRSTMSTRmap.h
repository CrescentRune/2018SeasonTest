/*
 * THRSTMSTRmap.h
 *
 *  Created on: Dec 11, 2017
 *      Author: WBI
 */

#ifndef SRC_THRSTMSTRMAP_H_
#define SRC_THRSTMSTRMAP_H_

enum ThrustJoystickAxis {
	xAxisJS, yAxisJS, rotateJS, throttleJS
};

enum ThrustJoystickButtons {
	trigger = 1,
	thumbSwitch,
	leftButton,
	rightButton,
	topLeftLeft,
	topLeftMiddle,
	topLeftRight,
	bottomLeftRight,
	bottomLeftMiddle,
	bottomLeftLeft,
	topRightRight,
	topRightMiddle,
	topRightLeft,
	bottomRightLeft,
	bottomRightMiddle,
	bottomRightRight
};

enum ThrustThrottleAxis {
	xAxisTR,
	yAxisTR,
	zAxisTR,
	deadXRotateTR,
	deadYRotateTR,
	weirdAwkwardButtonThingTR,
	sliderTR
};

enum ThrustThrottleButtons {
	rightThumbOrange,
	topLittleFingerButton,
	topRingFingerButton,
	flipOffTop,
	flipOffBottom,
	axisButton,
	middleHatUp,
	middleHatForward,
	middleHatDown,
	middleHatBackward,
	bottomHatUp,
	bottomHatForward,
	bottomHatDown,
	bottomHatBackward

};



#endif /* SRC_THRSTMSTRMAP_H_ */
