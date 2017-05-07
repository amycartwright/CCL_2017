#include "cclKinect.h"
#include "cclKinectNode.h"

VfxNodeCclKinect::VfxNodeCclKinect()
	: VfxNodeBase()
	, videoImage()
	, depthImage()
	, kinect(nullptr)
{
	kinect = new CclKinect();

	kinect->init();

	//

	resizeSockets(kInput_COUNT, kOutput_COUNT);
	addInput(kInput_DeviceId, kVfxPlugType_Int);
	addOutput(kOutput_VideoImage, kVfxPlugType_Image, &videoImage);
	addOutput(kOutput_DepthImage, kVfxPlugType_Image, &depthImage);
}

VfxNodeCclKinect::~VfxNodeCclKinect()
{
	kinect->shut();

	delete kinect;
	kinect = nullptr;	
}

void VfxNodeCclKinect::tick(const float dt)
{
	kinect->threadProcess();
	
	if (kinect->hasVideo)
	{
		// create texture from video data
		
		if (videoImage.texture != 0)
		{
			glDeleteTextures(1, &videoImage.texture);
		}
		
		if (kinect->bIsVideoInfrared)
			videoImage.texture = createTextureFromR8(kinect->videoData, kinect->width, kinect->height, true, true);
		else
			videoImage.texture = createTextureFromRGB8(kinect->videoData, kinect->width, kinect->height, true, true);
	}
	
	if (kinect->hasDepth)
	{
		// create texture from depth data
		
		if (depthImage.texture != 0)
		{
			glDeleteTextures(1, &depthImage.texture);
		}
		
		depthImage.texture = createTextureFromR8(kinect->depthData, kinect->width, kinect->height, true, true);
	}
}
