#pragma once

#include "libfreenect.h"
#include "Vec3.h"

struct CclKinect
{
	const static int width = 640;
	const static int height = 480;
	
	freenect_context * context;
	freenect_device * device;
	
	bool bIsVideoInfrared = false;
	bool bUseRegistration = false;
	
	void * videoData;
	void * depthData;
	
	bool hasVideo;
	bool hasDepth;
	
	freenect_led_options currentLed;
	bool ledIsDirty;
	
	float oldTiltAngle;
	float newTiltAngle;
	bool tiltAngleIsDirty;
	
	Vec3 mksAccel;
	Vec3 rawAccel;
	
	CclKinect()
		: context(nullptr)
		, device(nullptr)
		, videoData(nullptr)
		, depthData(nullptr)
		, hasVideo(false)
		, hasDepth(false)
		, currentLed(LED_GREEN)
		, ledIsDirty(true)
		, oldTiltAngle(0.f)
		, newTiltAngle(0.f)
		, tiltAngleIsDirty(true)
	{
	}
	
	bool init();
	bool shut();
	
	void threadInit();
	void threadShut();
	bool threadProcess();
	
	void threadMain();
	
	static void grabDepthFrame(freenect_device * dev, void * depth, uint32_t timestamp);
	static void grabVideoFrame(freenect_device * dev, void * video, uint32_t timestamp);
};
