#pragma once

#include <map>
#include <string>
#include <string.h>
#include <vector>

#include "vfxNodes/vfxNodeBase.h"

class Surface;

struct MotionPoint
{
	const std::string * name;
	
	float p[3];
	float v[3];
	
	MotionPoint()
	{
		memset(this, 0, sizeof(*this));
	}
};

struct MotionFrame
{
	static const int kNumPoints = 100;

	MotionPoint points[kNumPoints];
	int numPoints;
	
	MotionFrame()
		: points()
		, numPoints(0)
	{
	}
};

struct MotionData
{
	static const int kMaxFrames = 1024;

	MotionFrame frames[kMaxFrames];
	int nextWriteIndex;

	MotionData()
	{
		memset(frames, 0, sizeof(frames));
	}
};

struct MotionBankKey
{
	float time;
	float value;
	
	MotionBankKey()
		: time(0.f)
		, value(0.f)
	{
	}
	
	MotionBankKey(const float _time, const float _value)
		: time(_time)
		, value(_value)
	{
	}
};

struct MotionBankChannel
{
	std::vector<MotionBankKey> keys;
	int nextReadIndex;
	
	MotionBankChannel();
	
	void seek(const float time);
	float interp(const float time);
};

struct MotionBankJoint
{
	MotionBankChannel px;
	MotionBankChannel py;
	MotionBankChannel pz;
	
	MotionBankJoint()
		: px()
		, py()
		, pz()
	{
	}
};

struct MotionBankProvider
{
	std::map<std::string, MotionBankJoint> joints;
	
	MotionBankProvider()
		: joints()
	{
	}
	
	bool load(const char * filename);
	bool save(const char * filename) const;
	bool import(const char * filename);
	
	bool provide(const float time, MotionFrame & frame);
};

bool provideMotionData_MotionBank(const float time, MotionBankProvider & provider, MotionFrame & frame);
bool provideMotionData_Kinect(const float time, MotionFrame & frame);

struct VfxNodeCCL : VfxNodeBase
{
	enum Imnput
	{
		kInput_Filename,
		kInput_ShowJointNames,
		kInput_ReplaySpeed,
		kInput_VColorScale,
		kInput_BlurH,
		kInput_BlurV,
		kInput_COUNT
	};
	
	enum Output
	{
		kOutput_Image,
		kOutput_COUNT
	};
	
	MotionBankProvider provider;
	
	Surface * surface;
	
	VfxImage_Texture * outputImage;
	
	//
	
	std::string filename;
	
	float time;
	MotionFrame motionFrame;
	
	VfxNodeCCL();
	~VfxNodeCCL();
	
	virtual void tick(const float dt) override;
	virtual void draw() const override;
};
