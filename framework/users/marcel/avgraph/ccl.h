#pragma once

#include <map>
#include <string>
#include <string.h>
#include <vector>

#include "vfxNodes/vfxNodeBase.h"
#include "cclDancer.h"

class Surface;

struct CclKinect;

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

struct MotionFrameAnalysis
{
	float min[3];
	float max[3];
	
	float size[3];
	float sizeRcp[3];
	
	//
	
	float xOverY;
	
	//
	
	float wideness;
	float narrowness;
	float tallness;
	float smallness;
	
	MotionFrameAnalysis()
	{
		memset(this, 0, sizeof(*this));
	}
};

/*
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
*/

//

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

//

bool provideMotionData_MotionBank(const float time, MotionBankProvider & provider, MotionFrame & frame);
bool provideMotionData_Kinect(const float time, MotionFrame & frame);

struct VfxNodeCCL : VfxNodeBase
{
	const static int kNumDancers = 8;
	
	enum Imnput
	{
		kInput_Filename,
		kInput_ShowJointNames,
		kInput_ReplaySpeed,
		kInput_VColorScale,
		kInput_BlurH,
		kInput_BlurV,
		kInput_FixedJoint,
		kInput_UseOsc,
		kInput_OscTrigger,
		kInput_OscValues,
		kInput_OscScale,
		kInput_ShowGeneticDancers,
		kInput_VisualDancerBlendPerSecond,
        kInput_FitnessFunction,
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
	
	MotionFrame oscFrame;
	
	Mat4x4 frameTransform;
	
	Dancer currentDancer;
	Dancer fittestDancer;
	Dancer dancer[kNumDancers];
	double timeToNextGeneration;
	
	//
	
	std::string filename;
	
	float time;
	MotionFrame motionFrame;
	MotionFrameAnalysis analysis;
	
	VfxNodeCCL();
	~VfxNodeCCL();
	
	virtual void tick(const float dt) override;
	virtual void draw() const override;
	
	void analyzeFrame(MotionFrame & frame, MotionFrameAnalysis & analysis);
	
	void calculateNextGeneration();
	Dancer breed(const Dancer & o, const Dancer & d1, const Dancer & d2);
	void mutate(Dancer & d);
	
	virtual void handleTrigger(int socketIndex) override;
};
