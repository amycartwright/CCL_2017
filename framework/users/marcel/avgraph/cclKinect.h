#pragma once

#include "libfreenect.h"
#include "Vec3.h"

struct SDL_mutex;
struct SDL_Thread;

struct CclKinect
{
	const static int width = 640;
	const static int height = 480;
	
	freenect_context * context;
	freenect_device * device;
	
	bool bIsVideoInfrared = false;
	bool bUseRegistration = true;
	
	void * videoData[2];
	void * depthData[2];
	
	bool hasVideo;
	bool hasDepth;
	
	void * video;
	void * depth;
	
	freenect_led_options currentLed;
	bool ledIsDirty;
	
	float oldTiltAngle;
	float newTiltAngle;
	bool tiltAngleIsDirty;
	
	Vec3 mksAccel;
	Vec3 rawAccel;
	
	SDL_mutex * mutex;
	SDL_Thread * thread;
	bool stopThread;
	
	CclKinect()
		: context(nullptr)
		, device(nullptr)
		, videoData()
		, depthData()
		, hasVideo(false)
		, hasDepth(false)
		, video(nullptr)
		, depth(nullptr)
		, currentLed(LED_GREEN)
		, ledIsDirty(true)
		, oldTiltAngle(0.f)
		, newTiltAngle(0.f)
		, tiltAngleIsDirty(true)
		, mutex(nullptr)
		, thread(nullptr)
		, stopThread(false)
	{
	}
	
	bool init();
	bool shut();
	
	void threadInit();
	void threadShut();
	bool threadProcess();
	
	static int threadMain(void * userData);
	
	static void grabDepthFrame(freenect_device * dev, void * depth, uint32_t timestamp);
	static void grabVideoFrame(freenect_device * dev, void * video, uint32_t timestamp);
};
