#include "Utility.h"

bool debug_on = false;

swipeCallBackFunction Utility::swipeCallBack = NULL;
inputModeCallBackFunction Utility::inputModeCallBack = NULL;
/*void Utility::registerSwipeLeftCallBack(swipeLeftCallBackFunction userSwipeLeftFunction) {
	swipeLeftCallBack = userSwipeLeftFunction;
}

void Utility::registerSwipeUpCallBack(swipeUpCallBackFunction userSwipeUpFunction) {
	swipeUpCallBack = userSwipeUpFunction;
}*/

void Utility::registerSwipeCallBack(swipeCallBackFunction userSwipeFunction) {
	swipeCallBack = userSwipeFunction;
}

void Utility::registerInputModeCallBack(inputModeCallBackFunction userInputModeFunction) {
	inputModeCallBack = userInputModeFunction;
}

gesture_code::swipe_type Utility::computeSwipeDirection(Gesture gesture) {
	if (gesture.type() != Gesture::TYPE_SWIPE) {
		return gesture_code::swipe_type::INVALID_SWIPE;
	}
	gesture_code::swipe_type result = gesture_code::swipe_type::INVALID_SWIPE;
	SwipeGesture swipe = gesture;

	// Compare directions and give preference to the greatest linear movement.
	float fAbsX = std::fabs(swipe.direction().x);
	float fAbsY = std::fabs(swipe.direction().y);
	float fAbsZ = std::fabs(swipe.direction().z);

	// Was X the greatest?
	if (fAbsX > fAbsY && fAbsX > fAbsZ) {
		if (swipe.direction().x > 0) {
			result = gesture_code::swipe_type::RIGHT_SWIPE;
		}
		else {
			// swipeLeftCallBack(gesture_code::swipe_type::LEFT_SWIPE);
			result = gesture_code::swipe_type::LEFT_SWIPE;
		}
	}
	// Was Y the greatest?
	else if (fAbsY > fAbsX && fAbsY > fAbsZ) {
		if (swipe.direction().y > 0) {
			// swipeUpCallBack(gesture_code::swipe_type::UP_SWIPE);
			result = gesture_code::swipe_type::UP_SWIPE;
		}
		else {
			result = gesture_code::swipe_type::DOWN_SWIPE;
		}
	}
	else // Z was the greatest.
	{
		if (swipe.direction().z > 0) {
			result = gesture_code::swipe_type::BACKWARD_SWIPE;
		}
		else {
			result = gesture_code::swipe_type::FORWARD_SWIPE;
		}
	}

	if (swipeCallBack != NULL)
		swipeCallBack(result);
	return result;
}

bool Utility::computeInputMode(const Frame& frame) {
	Leap::Hand rightHand = frame.hands().rightmost();
	Leap::Hand leftHand = frame.hands().leftmost();

	unsigned int rightHandFingers = 0;
	unsigned int leftHandFingers = 0;
	unsigned int inputMode = input_code::input_type::INVALID_MODE;

	// check what fingers are extended in each hand
	for (unsigned int counter = 0; counter < NUMBER_OF_FINGERS; ++counter) {
		if (rightHand.fingers()[counter].isExtended())
			rightHandFingers += counter + 1;

		if (leftHand.fingers()[counter].isExtended())
			leftHandFingers += counter + 1;
	}

	if (debug_on) {
		std::cout << "Right Hand Fingers: " << rightHandFingers << std::endl;
		std::cout << "Left Hand Fingers: " << leftHandFingers << std::endl;
	}

	// deciding mode
	switch (leftHandFingers) {
		case input_code::LEFT_HAND_OPEN:
			switch (rightHandFingers) {
				case 5:
					inputMode = input_code::input_type::CURSOR_MODE;
					break;
				case 2:
					inputMode = input_code::input_type::DRAW_MODE;
					break;
				case 15:
					inputMode = input_code::input_type::TRACKBALL_MODE;
					break;
				default:
					break;
			}
			break;

		case input_code::LEFT_HAND_CLOSED:
			switch (rightHandFingers) {
				case 2:
					inputMode = input_code::input_type::ERASE_MODE;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	Leap::Vector rightPalmPosition = rightHand.palmPosition();
	Leap::Vector rightIndexFingerPosition = (rightHand.fingers())[1].tipPosition();

	if (debug_on) {
		std::cout << "Right Finger Position: (" << rightIndexFingerPosition.x << ", "
			<< rightIndexFingerPosition.y << ", " << rightIndexFingerPosition.z << ")" << std::endl;
		std::cout << "Right Hand Position: (" << rightPalmPosition.x << ", "
			<< rightPalmPosition.y << ", " << rightPalmPosition.z << ")" << std::endl;
		std::cout << "Mode: " << inputMode << std::endl;
	}

	if (inputModeCallBack != NULL)
		inputModeCallBack(inputMode, rightIndexFingerPosition.x, rightIndexFingerPosition.y, 
		rightIndexFingerPosition.z, rightPalmPosition.x, rightPalmPosition.y, rightPalmPosition.z);
	return inputMode;
}


Utility::~Utility() {
}
