#pragma once

#include "libfreenect.h"

struct CclKinect
{
	freenect_context * kinectContext;
	
	freenect_device * device;
	
	CclKinect()
		: kinectContext(nullptr)
		, device(nullptr)
	{
	}
	
	bool init();
	bool shut();
	
	void threadMain();
};
