#pragma once
#include "Leap.h"

using namespace Leap;

const unsigned int NUMBER_OF_FINGERS = 5;

namespace gesture_code {
	enum swipe_type {
		FORWARD_SWIPE,
		BACKWARD_SWIPE,
		UP_SWIPE,
		DOWN_SWIPE,
		LEFT_SWIPE,
		RIGHT_SWIPE,
		INVALID_SWIPE
	};
};

namespace input_code {
	enum input_type {
		CURSOR_MODE,
		DRAW_MODE,
		ERASE_MODE,
		TRACKBALL_MODE,
		INVALID_MODE
	};

	const unsigned int LEFT_HAND_OPEN = 15;
	const unsigned int LEFT_HAND_CLOSED = 0;
}

typedef void(*swipeCallBackFunction)(gesture_code::swipe_type swipe_direction);
typedef void(*inputModeCallBackFunction)
(unsigned int mode, float finger_x, float finger_y, float finger_z,
float palm_x, float palm_y, float palm_z);

class Utility {
	public:
	static swipeCallBackFunction swipeCallBack;
	static inputModeCallBackFunction inputModeCallBack;

	Utility();
	static void registerSwipeCallBack(swipeCallBackFunction userSwipeUpFunction);
	static void registerInputModeCallBack(inputModeCallBackFunction userInputModeFunction);
	static gesture_code::swipe_type computeSwipeDirection(Gesture gesture);
	static bool computeInputMode(const Frame& frame);
	~Utility();

	/*typedef void (*swipeLeftCallBackFunction)(gesture_code::swipe_type swipe_direction);
	typedef void (*swipeUpCallBackFunction)(gesture_code::swipe_type swipe_direction);*/

	/*typedef void (*cursorModeCallBackFunction)
	(float finger_x, float finger_y, float finger_z, float palm_x, float palm_y, float palm_z);
	typedef void(*drawModeCallBackFunction)
	(float finger_x, float finger_y, float finger_z, float palm_x, float palm_y, float palm_z);
	typedef void(*trackBallModeCallBackFunction)
	(float finger_x, float finger_y, float finger_z, float palm_x, float palm_y, float palm_z);
	typedef void(*eraseModeCallBackFunction)
	(float finger_x, float finger_y, float finger_z, float palm_x, float palm_y, float palm_z);*/

	/*swipeLeftCallBackFunction swipeLeftCallBack;
	swipeUpCallBackFunction swipeUpCallBack;*/

	/*cursorModeCallBackFunction cursorModeCallBack;
	drawModeCallBackFunction drawModeCallBack;
	trackBallModeCallBackFunction trackBallModeCallBack;
	eraseModeCallBackFunction eraseModeCallBack;*/

	/*void registerSwipeLeftCallBack(swipeLeftCallBackFunction userSwipeLeftFunction);
	void registerSwipeUpCallBack(swipeUpCallBackFunction userSwipeUpFunction);*/

	/*void registerCursorModeCallBack(cursorModeCallBackFunction userCursorModeCallBack);
	void registerDrawModeCallBack(drawModeCallBackFunction userDrawModeCallBack);
	void registerTrackballModeCallBack(trackBallModeCallBackFunction userTrackballModeCallBack);
	void registerEraseModeCallBack(eraseModeCallBackFunction userEraseModeCallBack);*/
};

