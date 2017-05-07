#pragma once

#include "vfxNodes/vfxNodeBase.h"

struct CclKinect;

struct VfxNodeCclKinect : VfxNodeBase
{
	enum Input
	{
		kInput_DeviceId,
		kInput_COUNT
	};
	
	enum Output
	{
		kOutput_VideoImage,
		kOutput_DepthImage,
		kOutput_COUNT
	};

	VfxImage_Texture videoImage;
	VfxImage_Texture depthImage;
	
	CclKinect * kinect;

	VfxNodeCclKinect();
	virtual ~VfxNodeCclKinect() override;
	
	virtual void tick(const float dt) override;
};
