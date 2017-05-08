#include "cclKinect.h"
#include "cclKinectNode.h"

VfxNodeCclKinect::VfxNodeCclKinect()
	: VfxNodeBase()
	, videoImage()
	, depthImage()
	, kinect(nullptr)
{
	resizeSockets(kInput_COUNT, kOutput_COUNT);
	addInput(kInput_DeviceId, kVfxPlugType_Int);
	addInput(kInput_Infrared, kVfxPlugType_Bool);
	addOutput(kOutput_VideoImage, kVfxPlugType_Image, &videoImage);
	addOutput(kOutput_DepthImage, kVfxPlugType_Image, &depthImage);
}

VfxNodeCclKinect::~VfxNodeCclKinect()
{
	kinect->shut();

	delete kinect;
	kinect = nullptr;	
}

void VfxNodeCclKinect::init(const GraphNode & node)
{
	const bool videoIsInfrared = getInputBool(kInput_Infrared, false);
	
	kinect = new CclKinect();
	kinect->bIsVideoInfrared = videoIsInfrared;
	
	kinect->init();
}

void VfxNodeCclKinect::tick(const float dt)
{
	SDL_LockMutex(kinect->mutex);
	{
		if (kinect->hasVideo)
		{
			kinect->hasVideo = false;
			
			// create texture from video data
			
			if (videoImage.texture != 0)
			{
				glDeleteTextures(1, &videoImage.texture);
			}
			
			if (kinect->bIsVideoInfrared)
			{
				videoImage.texture = createTextureFromR8(kinect->video, kinect->width, kinect->height, true, true);
				
				glBindTexture(GL_TEXTURE_2D, videoImage.texture);
				GLint swizzleMask[4] = { GL_RED, GL_RED, GL_RED, GL_ONE };
				glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
			}
			else
				videoImage.texture = createTextureFromRGB8(kinect->video, kinect->width, kinect->height, true, true);
		}
		
		if (kinect->hasDepth)
		{
			kinect->hasDepth = false;
			
			// FREENECT_DEPTH_MM_MAX_VALUE
			// FREENECT_DEPTH_MM_NO_VALUE
			
			// create texture from depth data
			
			if (depthImage.texture != 0)
			{
				glDeleteTextures(1, &depthImage.texture);
			}
			
			depthImage.texture = createTextureFromR16(kinect->depth, kinect->width, kinect->height, true, true);
			
			glBindTexture(GL_TEXTURE_2D, depthImage.texture);
			GLint swizzleMask[4] = { GL_RED, GL_RED, GL_RED, GL_ONE };
			glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
		}
	}
	SDL_UnlockMutex(kinect->mutex);
}
