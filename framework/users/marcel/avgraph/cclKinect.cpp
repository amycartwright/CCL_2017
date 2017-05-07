#include "cclKinect.h"
#include "framework.h"

#include "libfreenect_registration.h"
#include "freenect_internal.h" // for access to freenect_device.registration.zero_plane_info

const static int width = 640;
const static int height = 480;

bool bIsVideoInfrared = false;
bool bUseRegistration = false;

static void grabDepthFrame(freenect_device * dev, void * depth, uint32_t timestamp)
{
	logDebug("got depth frame: %u", timestamp);
	
#if 0
	framework.process();
	
	framework.beginDraw(0, 0, 0, 0);
	{
		GLuint texture = createTextureFromR8(depth, width, height, true, true);
		
		const int x1 = 0;
		const int y1 = 0;
		const int x2 = width;
		const int y2 = height;
		
		gxSetTexture(texture);
		gxBegin(GL_QUADS);
		{
			gxTexCoord2f(1.f, 0.f); gxVertex2f(x1, y1);
			gxTexCoord2f(0.f, 0.f); gxVertex2f(x2, y1);
			gxTexCoord2f(0.f, 1.f); gxVertex2f(x2, y2);
			gxTexCoord2f(1.f, 1.f); gxVertex2f(x1, y2);
		}
		gxEnd();
		gxSetTexture(0);
		
		glDeleteTextures(1, &texture);
		texture = 0;
	}
	framework.endDraw();
#endif
}

static void grabVideoFrame(freenect_device * dev, void * video, uint32_t timestamp)
{
	logDebug("got video frame: %u", timestamp);
	
#if 1
	framework.process();
	
	framework.beginDraw(0, 0, 0, 0);
	{
		GLuint texture;
		
		if (bIsVideoInfrared)
			texture = createTextureFromR8(video, width, height, true, true);
		else
			texture = createTextureFromRGB8(video, width, height, true, true);
		
		const int x1 = 0;
		const int y1 = 0;
		const int x2 = width;
		const int y2 = height;
		
		gxSetTexture(texture);
		gxBegin(GL_QUADS);
		{
			gxTexCoord2f(1.f, 0.f); gxVertex2f(x1, y1);
			gxTexCoord2f(0.f, 0.f); gxVertex2f(x2, y1);
			gxTexCoord2f(0.f, 1.f); gxVertex2f(x2, y2);
			gxTexCoord2f(1.f, 1.f); gxVertex2f(x1, y2);
		}
		gxEnd();
		gxSetTexture(0);
		
		glDeleteTextures(1, &texture);
		texture = 0;
	}
	framework.endDraw();
#endif
}

bool CclKinect::init()
{
	if (freenect_init(&kinectContext, nullptr) < 0)
	{
		logError(")freenect_init failed");
		return false;
	}
    
    #ifdef OFX_KINECT_EXTRA_FW
        freenect_set_fw_address_nui(kinectContext, ofxKinectExtras::getFWData1473(), ofxKinectExtras::getFWSize1473());
        freenect_set_fw_address_k4w(kinectContext, ofxKinectExtras::getFWDatak4w(), ofxKinectExtras::getFWSizek4w());
    #endif
    
	freenect_set_log_level(kinectContext, FREENECT_LOG_WARNING);
	freenect_select_subdevices(kinectContext, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
	
	//
	
	std::string serial;
	
	//
	
	freenect_device_attributes * devAttrib = nullptr;
	
	const int numDevices = freenect_list_device_attributes(kinectContext, &devAttrib);
	
	for (int i = 0; i < numDevices; ++i)
	{
		logDebug("camera serial: %s", devAttrib->camera_serial);
		
		serial = devAttrib->camera_serial;
		
		devAttrib = devAttrib->next;
	}
	
	freenect_free_device_attributes(devAttrib);
	devAttrib = nullptr;
	
	//
	
	bool deviceHasMotorControl = false;
	
	if (!serial.empty())
	{
		if (freenect_open_device_by_camera_serial(kinectContext, &device, serial.c_str()) < 0)
		{
			logError("failed to open camera");
			return false;
		}
		
		if (serial == "0000000000000000")
		{
			//if we do motor control via the audio device ( ie: 1473 or k4w ) and we have firmware uploaded
			//then we can do motor stuff! :)
			if (device->motor_control_with_audio_enabled)
			{
				deviceHasMotorControl = true;
			}
			else
			{
				logDebug("open device: device does not have motor control");
			}
		}
		else
		{
			deviceHasMotorControl = true;
		}
		
		//We have to do this as freenect has 488 pixels for the IR image height.
		//Instead of having slightly different sizes depending on capture we will crop the last 8 rows of pixels which are empty.
		int videoHeight = height;
		if (bIsVideoInfrared)
			videoHeight = 488;
	
		std::vector<uint8_t> depthPixels;
		std::vector<uint8_t> videoPixels;
		depthPixels.resize(width * height);
		videoPixels.resize(width * height * (bIsVideoInfrared ? 1 : 3)); // infrared. otherwise 3
		
		freenect_set_user(device, this);
		freenect_set_depth_buffer(device, &depthPixels[0]);
		freenect_set_video_buffer(device, &videoPixels[0]);
		freenect_set_depth_callback(device, &grabDepthFrame);
		freenect_set_video_callback(device, &grabVideoFrame);
		
		threadMain();
	}
	
	return true;
}

bool CclKinect::shut()
{
	freenect_close_device(device);
	device = nullptr;
	
	freenect_shutdown(kinectContext);
	kinectContext = nullptr;
	
	return true;
}

void CclKinect::threadMain()
{
	bool bGrabVideo = true;
	
	freenect_led_options currentLed = LED_GREEN;
	bool bLedNeedsApplying = false;
	
	float currentTiltAngleDeg = 0.f;
	float targetTiltAngleDeg = 0.f;
	bool bTiltNeedsApplying = false;
	Vec3 mksAccel;
	Vec3 rawAccel;
	
	freenect_set_led(device, currentLed);
	
	const freenect_frame_mode videoMode = freenect_find_video_mode(
		FREENECT_RESOLUTION_MEDIUM,
		bIsVideoInfrared
		? FREENECT_VIDEO_IR_8BIT
		: FREENECT_VIDEO_RGB);
	
	freenect_set_video_mode(device, videoMode);
	
	const freenect_frame_mode depthMode = freenect_find_depth_mode(
		FREENECT_RESOLUTION_MEDIUM,
		bUseRegistration
		? FREENECT_DEPTH_REGISTERED
		: FREENECT_DEPTH_MM);
	
	freenect_set_depth_mode(device, depthMode);

	freenect_start_depth(device);
	
	if (bGrabVideo)
	{
		freenect_start_video(device);
	}
	
	bool stop = false;
	
	while (!stop && freenect_process_events(kinectContext) >= 0)
	{
		if (bTiltNeedsApplying)
		{
			freenect_set_tilt_degs(device, targetTiltAngleDeg);
			
			bTiltNeedsApplying = false;
		}
		
		if (bLedNeedsApplying)
		{
			freenect_set_led(device, currentLed);
			
			bLedNeedsApplying = false;
		}

		freenect_update_tilt_state(device);
		freenect_raw_tilt_state * tilt = freenect_get_tilt_state(device);
		currentTiltAngleDeg = freenect_get_tilt_degs(tilt);

		rawAccel = Vec3(tilt->accelerometer_x, tilt->accelerometer_y, tilt->accelerometer_z);

		double dx,dy,dz;
		freenect_get_mks_accel(tilt, &dx, &dy, &dz);
		mksAccel = Vec3(dx, dy, dz);
	}
    
	// finish up a tilt on exit
	if (bTiltNeedsApplying)
	{
		freenect_set_tilt_degs(device, targetTiltAngleDeg);
		bTiltNeedsApplying = false;
	}
    
	freenect_stop_depth(device);
	freenect_stop_video(device);
	
	freenect_set_led(device, LED_RED);
    
	//kinectContext.close(*this);
}