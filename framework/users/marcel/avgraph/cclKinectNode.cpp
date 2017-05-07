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
		{
			videoImage.texture = createTextureFromR8(kinect->videoData, kinect->width, kinect->height, true, true);
			
			glBindTexture(GL_TEXTURE_2D, videoImage.texture);
			GLint swizzleMask[4] = { GL_RED, GL_RED, GL_RED, GL_ONE };
			glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
		}
		else
			videoImage.texture = createTextureFromRGB8(kinect->videoData, kinect->width, kinect->height, true, true);
	}
	
	if (kinect->hasDepth)
	{
		// FREENECT_DEPTH_MM_MAX_VALUE
		// FREENECT_DEPTH_MM_NO_VALUE
		
		// create texture from depth data
		
		if (depthImage.texture != 0)
		{
			glDeleteTextures(1, &depthImage.texture);
		}
		
		depthImage.texture = createTextureFromR16(kinect->depthData, kinect->width, kinect->height, true, true);
		
		glBindTexture(GL_TEXTURE_2D, depthImage.texture);
		GLint swizzleMask[4] = { GL_RED, GL_RED, GL_RED, GL_ONE };
		glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
	}
}
