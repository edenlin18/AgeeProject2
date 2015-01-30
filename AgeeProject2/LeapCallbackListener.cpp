#include "LeapCallbackListener.h"

bool debug = false;

void LeapCallbackListener::onConnect(const Controller& controller) {
	std::cout << "Connected" << std::endl;
	controller.enableGesture(Gesture::TYPE_SWIPE);
}

void LeapCallbackListener::onFrame(const Controller& controller) {
	// std::cout << "Frame available" << std::endl;
	HandList hands = frame.hands();

	// check if it is the very first frame
	if (!frame.isValid())
		frame = controller.frame();

	// check for swipe gesture
	if (hands.count() == 2 && hands[0].isRight() &&
		(frame.gestures())[0].type() == Gesture::Type::TYPE_INVALID &&
		(controller.frame().gestures())[0].type() == Gesture::Type::TYPE_SWIPE) {
		unsigned int swipeDirection = Utility::computeSwipeDirection(controller.frame().gestures()[0]);
		if (debug) {
			std::cout << "Swipe Direction: " << swipeDirection << std::endl;
		}
	}
	// check for input mode
	else if (hands.count() == 1) {
		unsigned int input_mode = Utility::computeInputMode(frame);
		if (debug) {
			std::cout << "This input is: " << input_mode << std::endl;
		}
	}

	frame = controller.frame();
}

LeapCallbackListener::~LeapCallbackListener() {
}
