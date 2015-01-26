#pragma once
#include <iostream>
#include <string.h>
#include "Leap.h"
#include "Utility.h"

using namespace Leap;

class LeapCallbackListener : public Listener {
	public:
	Frame frame;
	virtual void onConnect(const Controller&);
	virtual void onFrame(const Controller&);
	~LeapCallbackListener();
};

