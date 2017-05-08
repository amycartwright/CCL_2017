#include "ccl.h"
#include "framework.h"
#include "json.hpp"

#include "FileStream.h"
#include "StreamReader.h"
#include "StreamWriter.h"

#include "Calc.h"
#include "Parse.h"
#include "StringEx.h"

#include "../libparticle/ui.h"

#include "cclKinect.h"

extern const int GFX_SX;
extern const int GFX_SY;

extern void splitString(const std::string & str, std::vector<std::string> & result, char c);

//

using json = nlohmann::json;

//

static int xIndex = 0;
static int yIndex = 2;

//

MotionBankChannel::MotionBankChannel()
	: keys()
	, nextReadIndex()
{
}

void MotionBankChannel::seek(const float time)
{
	while (nextReadIndex - 1 > 0 && time < keys[nextReadIndex].time)
		nextReadIndex--;
	
	while (nextReadIndex + 1 < keys.size() && time >= keys[nextReadIndex + 1].time)
		nextReadIndex++;
}

float MotionBankChannel::interp(const float time)
{
	if (nextReadIndex + 1 == keys.size())
	{
		return keys[nextReadIndex].value;
	}
	else
	{
		const float value1 = keys[nextReadIndex + 0].value;
		const float value2 = keys[nextReadIndex + 1].value;
		const float time1 = keys[nextReadIndex + 0].time;
		const float time2 = keys[nextReadIndex + 1].time;
		
		const float t2 = (time - time1) / (time2 - time1);
		const float t1 = 1.f - t2;
		
		return value1 * t1 + value2 * t2;
	}
}

//

static int intValue(json::reference j, const char * name, int defaultValue)
{
	auto v = j[name];
	if (!v.is_number_integer())
		return defaultValue;
	else
		return v.get<int>();
}

static std::string stringValue(json::reference j, const char * name, const char * defaultValue)
{
	auto v = j[name];
	if (!v.is_string())
		return defaultValue;
	else
		return v.get<std::string>();
}

static void readChannel(MotionBankChannel & channel, StreamReader & reader)
{
	const int numKeys = reader.ReadInt32();
	
	channel.keys.resize(numKeys);
	
	reader.ReadBytes(&channel.keys[0], sizeof(MotionBankKey) * channel.keys.size());
}

static void writeChannel(const MotionBankChannel & channel, StreamWriter & writer)
{
	writer.WriteInt32(channel.keys.size());
	
	writer.WriteBytes(&channel.keys[0], sizeof(MotionBankKey) * channel.keys.size());
}

bool MotionBankProvider::load(const char * filename)
{
	//return false;
	
	try
	{
		joints.clear();
		
		//
		
		FileStream stream;
		stream.Open(filename, OpenMode_Read);
		StreamReader reader(&stream, false);
		
		const int numJoints = reader.ReadInt32();
		
		if (numJoints <= 0)
			return false;
		
		for (int i = 0; i < numJoints; ++i)
		{
			char name[256];
			reader.ReadText_Binary(name, sizeof(name));
			
			MotionBankJoint & joint = joints[name];
			
			readChannel(joint.px, reader);
			readChannel(joint.py, reader);
			readChannel(joint.pz, reader);
		}
		
		return true;
	}
	catch (std::exception & e)
	{
		logError(e.what());
		
		return false;
	}
}

bool MotionBankProvider::save(const char * filename) const
{
	try
	{
		FileStream stream;
		stream.Open(filename, OpenMode_Write);
		StreamWriter writer(&stream, false);
		
		writer.WriteInt32(joints.size());
		
		for (auto & nameAndJoint : joints)
		{
			const std::string & name = nameAndJoint.first;
			const MotionBankJoint & joint = nameAndJoint.second;
			
			writer.WriteText_Binary(name);
			
			writeChannel(joint.px, writer);
			writeChannel(joint.py, writer);
			writeChannel(joint.pz, writer);
		}
		
		return true;
	}
	catch (std::exception & e)
	{
		logError(e.what());
		
		return false;
	}
}

bool MotionBankProvider::import(const char * filename)
{
	try
	{
		FileStream file;
		file.Open(filename, (OpenMode)(OpenMode_Read | OpenMode_Text));
		
		StreamReader reader(&file, false);
		uint8_t * bytes = reader.ReadAllBytes();
		char * text = (char*)bytes;
		char * BLHip = strstr(text, "BLHip");
		BLHip -= 16;
		json j = json::parse(text);
		delete[] bytes;
		
		auto channelsItr = j.find("channels");
		
		if (channelsItr != j.end())
		{
			std::cout << "found channels!" << std::endl;
			
			auto & channels = channelsItr.value();
			
			for (auto channel : channels)
			{
				auto title = channel["title"].get<std::string>();
				
				std::cout << "channel: " << title << std::endl;
				
				std::cout << channel.type_name() << std::endl;
				
				auto streamsItr = channel.find("streams");
				
				if (streamsItr != channel.end())
				{
					auto & streams = streamsItr.value();
					
					std::cout << "found streams!" << std::endl;
					
					for (auto stream : streams)
					{
						//std::cout << "Stream.." << std::endl;
						
						auto title = stringValue(stream, "title", "");
						auto group = stringValue(stream, "group", "");
						auto frameCount = intValue(stream, "frameCount", 0);
						auto fps = intValue(stream, "fps", 0);
						
						std::cout << "stream: " << title << ", group: " << group << ", frameCount: " << frameCount << ", fps: " << fps << std::endl;
						
						if (title.empty())
						{
							logWarning("title not set. skipping!");
							continue;
						}
						
						if (group.empty())
						{
							logWarning("group not set. skipping!");
							continue;
						}
						
						if (frameCount <= 0)
						{
							logWarning("frameCount is zero. skipping!");
							continue;
						}
						
						if (fps <= 0)
						{
							logWarning("fps is zero. skipping!");
							continue;
						}
						
						if (title != "X" && title != "Y" && title != "Z")
						{
							logWarning("joint channel unknown. skipping!");
							continue;
						}
						
						//
						
						if (String::StartsWith(group, "V_"))
						{
							logDebug("skipping empty joint. it only contains zeroes! joint: %s", group.c_str());
							continue;
						}
						
						//
						
						MotionBankJoint & joint = joints[group];
						
						MotionBankChannel * jointChannel = nullptr;
						
						if (title == "X")
							jointChannel = &joint.px;
						if (title == "Y")
							jointChannel = &joint.py;
						if (title == "Z")
							jointChannel = &joint.pz;
						
						jointChannel->keys.resize(frameCount);
						
						auto framesItr = stream.find("frames");
						
						if (framesItr != stream.end())
						{
							auto & frames = framesItr.value();
							
							//std::cout << "found frames!" << std::endl;
							
							int index = 0;
							
							float lastValue = 0.f;
							
							for (auto frame : frames)
							{
								float value;
								
								if (!frame.is_number())
								{
									//logDebug("value is not a number. using last value");
									
									value = lastValue;
								}
								else
								{
									value = frame.get<float>();
									
									lastValue = value;
								}
								
								//std::cout << index << " : " << value << std::endl;
								
								if (index < frameCount)
								{
									jointChannel->keys[index].time = index / float(fps);
									jointChannel->keys[index].value = value;
								}
								
								index++;
							}
						}
					}
				}
				
				break; // only go for first channel
			}
		}
	}
	catch (std::exception & e)
	{
		logError(e.what());
		return false;
	}
	
	return true;
}

bool MotionBankProvider::provide(const float time, MotionFrame & frame)
{
	int index = 0;
	
	for (auto & nameAndJoint : joints)
	{
		if (frame.numPoints < frame.kNumPoints)
		{
			auto & name = nameAndJoint.first;
			auto & joint = nameAndJoint.second;
			
			frame.points[index].name = &name;
			
			joint.px.seek(time);
			joint.py.seek(time);
			joint.pz.seek(time);
			
			frame.points[index].p[0] = joint.px.interp(time);
			frame.points[index].p[1] = joint.py.interp(time);
			frame.points[index].p[2] = joint.pz.interp(time);
			
			const float eps = .01f;
			
			frame.points[index].v[0] = (joint.px.interp(time + eps) - joint.px.interp(time)) / eps;
			frame.points[index].v[1] = (joint.py.interp(time + eps) - joint.py.interp(time)) / eps;
			frame.points[index].v[2] = (joint.pz.interp(time + eps) - joint.pz.interp(time)) / eps;
			
			frame.numPoints++;
			
			index++;
		}
	}
	
	return true;
}

//

bool provideMotionData_MotionBank(const float time, MotionBankProvider & provider, MotionFrame & frame)
{
	return provider.provide(time, frame);
}

bool provideMotionData_Kinect(const float time, MotionFrame & frame)
{
	return false;
}

//

VfxNodeCCL::VfxNodeCCL()
	: VfxNodeBase()
	, provider()
	, surface(nullptr)
	, outputImage(nullptr)
	, filename()
	, time(0.f)
	, motionFrame()
{
	surface = new Surface(GFX_SX, GFX_SY, false);
	
	outputImage = new VfxImage_Texture();
	
	//
	
	resizeSockets(kInput_COUNT, kOutput_COUNT);
	addInput(kInput_Filename, kVfxPlugType_String);
	addInput(kInput_ShowJointNames, kVfxPlugType_Bool);
	addInput(kInput_ReplaySpeed, kVfxPlugType_Float);
	addInput(kInput_VColorScale, kVfxPlugType_Float);
	addInput(kInput_BlurH, kVfxPlugType_Float);
	addInput(kInput_BlurV, kVfxPlugType_Float);
	addInput(kInput_FixedJoint, kVfxPlugType_Int);
	addInput(kInput_UseOsc, kVfxPlugType_Bool);
	addInput(kInput_OscTrigger, kVfxPlugType_Trigger);
	addInput(kInput_OscValues, kVfxPlugType_String);
	addInput(kInput_OscScale, kVfxPlugType_Float);
	addOutput(kOutput_Image, kVfxPlugType_Image, outputImage);
	
#if 0 // todo : properly integrate Kinect
	// test kinect
	
	CclKinect kinect;
	
	if (kinect.init())
	{
		kinect.shut();
	}
#endif
}

VfxNodeCCL::~VfxNodeCCL()
{
	delete outputImage;
	outputImage = nullptr;
	
	delete surface;
	surface = nullptr;
}

void VfxNodeCCL::tick(const float dt)
{
	const char * newFilename = getInputString(kInput_Filename, "");
	const bool useOsc = getInputBool(kInput_UseOsc, false);
	
	// reload data, if necessary
	
	if (newFilename != filename)
	{
		filename = newFilename;
		
		std::string cachedFilename = filename + ".cache";
		
		if (!provider.load(cachedFilename.c_str()))
		{
			if (provider.import(filename.c_str()))
			{
				provider.save(cachedFilename.c_str());
			}
		}
	}
	
	// update the animation
	
	const float replaySpeed = getInputFloat(kInput_ReplaySpeed, 1.f);
	
	time += dt * replaySpeed;
	
	motionFrame = MotionFrame();
	
	Mat4x4 transform;
	
	if (useOsc)
	{
		motionFrame = oscFrame;
		
		// transform the points into the desired coordinate frame
		
		const float s = .2f;
		const float d2r = Calc::DegToRad(1.f);
		
		//transform = Mat4x4(true).Scale(-1, 1, 1).RotateX(d2r * 90.f).RotateY(d2r * 180.f).Scale(s, s, s);
		transform = Mat4x4(true);
		
		xIndex = 0;
		yIndex = 1;
	}
	else
	{
		provideMotionData_MotionBank(time, provider, motionFrame);
		
		const float s = .2f;
		const float d2r = Calc::DegToRad(1.f);
		
		transform = Mat4x4(true).RotateX(d2r * 90.f).RotateY(d2r * 180.f).Scale(s, s, s);
		
		xIndex = 0;
		yIndex = 2;
	}
	
	// transform the points into the desired coordinate frame
	
	for (int i = 0; i < motionFrame.numPoints; ++i)
	{
		MotionPoint & mp = motionFrame.points[i];
		
		const Vec3 p = transform * Vec3(mp.p[0], mp.p[1], mp.p[2]);
		
		mp.p[0] = p[0];
		mp.p[1] = p[1];
		mp.p[2] = p[2];
	}
	
	// run analysis on the frame we just captured
	
	analyzeFrame(motionFrame, analysis);
	
	//
	
	outputImage->texture = surface->getTexture();
}

void VfxNodeCCL::draw() const
{
	const bool showJointNames = getInputBool(kInput_ShowJointNames, false);
	const float vColorScale = getInputFloat(kInput_VColorScale, 1.f);
	const float blurH = getInputFloat(kInput_BlurH, 0.f);
	const float blurV = getInputFloat(kInput_BlurV, 0.f);
	const int fixedJoint = getInputInt(kInput_FixedJoint, -1);
	
	pushSurface(surface);
	{
		surface->clear(0, 0, 0);
		
		setColor(127, 127, 127);
		//drawUiRectCheckered(0, 0, GFX_SX, GFX_SY, 32.f);
		
		gxPushMatrix();
		{
			gxTranslatef(GFX_SX/2, GFX_SY/2, 1.f);
			
			if (fixedJoint >= 0 && fixedJoint < motionFrame.numPoints)
			{
				const MotionPoint & mp = motionFrame.points[fixedJoint];
				gxTranslatef(-mp.p[xIndex], -mp.p[yIndex], 0.f);
			}
			
			hqBegin(HQ_FILLED_CIRCLES);
			{
				for (int i = 0; i < motionFrame.numPoints; ++i)
				{
					const MotionPoint & mp = motionFrame.points[i];
					
					//setColor(colorWhite);
					setColorf(mp.v[0] * vColorScale + .5f, mp.v[1] * vColorScale + .5f, mp.v[2] * vColorScale + .5f);
					hqFillCircle(mp.p[xIndex], mp.p[yIndex], 5.f);
				}
			}
			hqEnd();
			
			if (showJointNames)
			{
				setFont("calibri.ttf");
				setColor(colorWhite);
				for (int i = 0; i < motionFrame.numPoints; ++i)
				{
					const MotionPoint & mp = motionFrame.points[i];
					
					if (mp.name)
					{
						drawText(mp.p[xIndex], mp.p[yIndex], 12, 0, 0, "%s", mp.name->c_str());
					}
				}
				
				if (fixedJoint >= 0 && fixedJoint < motionFrame.numPoints)
				{
					const MotionPoint & mp = motionFrame.points[fixedJoint];
					
					if (mp.name)
						drawText(20, 20, 24, 0, 0, "%s", mp.name->c_str());
				}
			}
		}
		gxPopMatrix();
		
	#if 1
		gxPushMatrix();
		{
			gxTranslatef(GFX_SX/2, GFX_SY*3/5, 1.f);
			
			setFont("calibri.ttf");
			int y = 0;
			
			setColor(colorGreen);
			drawText(0, y, 18, 0.f, 0.f, "size[x]: %f", analysis.size[xIndex]);
			y += 20;
			drawText(0, y, 18, 0.f, 0.f, "size[y]: %f", analysis.size[yIndex]);
			y += 20;
			y += 20;
			
			setColorf(1, 1, 1, analysis.wideness);
			drawText(0, y, 18, 0.f, 0.f, "wideness: %f", analysis.wideness);
			y += 20;
			setColorf(1, 1, 1, analysis.narrowness);
			drawText(0, y, 18, 0.f, 0.f, "narrowness: %f", analysis.narrowness);
			y += 20;
			
			setColorf(1, 1, 1, analysis.tallness);
			drawText(0, y, 18, 0.f, 0.f, "tallness: %f", analysis.tallness);
			y += 20;
			setColorf(1, 1, 1, analysis.smallness);
			drawText(0, y, 18, 0.f, 0.f, "smallness: %f", analysis.smallness);
			y += 20;
		}
		gxPopMatrix();
	#endif
	}
	popSurface();
	
	pushBlend(BLEND_OPAQUE);
	if (blurH > 0.f)
	{
		setShader_GaussianBlurH(surface->getTexture(), 64, blurH);
		surface->postprocess();
		clearShader();
	}
	if (blurV > 0.f)
	{
		setShader_GaussianBlurV(surface->getTexture(), 64, blurV);
		surface->postprocess();
		clearShader();
	}
	popBlend();
	
	outputImage->texture = surface->getTexture();
}

void VfxNodeCCL::analyzeFrame(MotionFrame & frame, MotionFrameAnalysis & analysis)
{
	const float eps = .0001f;
	
	// todo : calculate metrics related to posture, movement speed, and whatnot
	
	analysis = MotionFrameAnalysis();
	
	if (frame.numPoints > 0)
	{
		// calculate bounding box
		
		float min[3];
		float max[3];
		
		for (int i = 0; i < 3; ++i)
		{
			min[i] = frame.points[0].p[i];
			max[i] = frame.points[0].p[i];
		}
		
		for (int i = 1; i < frame.numPoints; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				min[j] = std::min(min[j], frame.points[i].p[j]);
				max[j] = std::max(max[j], frame.points[i].p[j]);
			}
		}
		
		const float kWidenessBegin = 100.f;
		const float kWidenessEnd = 200.f;
		const float kNarrownessBegin = 100.f;
		const float kNarrownessEnd = 50.f;
		const float kTallnessBegin = 300.f;
		const float kTallnessEnd = 360.f;
		const float kSmallnessBegin = 300.f;
		const float kSmallnessEnd = 200.f;
		
		for (int i = 0; i < 3; ++i)
		{
			analysis.min[i] = min[i];
			analysis.max[i] = max[i];
			
			analysis.size[i] = max[i] - min[i];
			analysis.sizeRcp[i] = 1.f / (max[i] - min[i] + eps);
			
			analysis.xOverY = analysis.size[xIndex] / (analysis.size[yIndex] + eps);
			
			analysis.wideness = std::max(0.f, std::min(1.f, (analysis.size[xIndex] - kWidenessBegin) / (kWidenessEnd - kWidenessBegin)));
			analysis.narrowness = std::max(0.f, std::min(1.f, (analysis.size[xIndex] - kNarrownessBegin) / (kNarrownessEnd - kNarrownessBegin)));
			
			analysis.tallness = std::max(0.f, std::min(1.f, (analysis.size[yIndex] - kTallnessBegin) / (kTallnessEnd - kTallnessBegin)));
			analysis.smallness = std::max(0.f, std::min(1.f, (analysis.size[yIndex] - kSmallnessBegin) / (kSmallnessEnd - kSmallnessBegin)));
		}
	}
}

void VfxNodeCCL::handleTrigger(int socketIndex)
{
	if (socketIndex == kInput_OscTrigger)
	{
		const std::string values = getInputString(kInput_OscValues, "");
		const float oscScale = getInputFloat(kInput_OscScale, 1.f);
		
		std::vector<std::string> splitValues;
		
		splitString(values, splitValues, ',');
		
		std::vector<float> floatValues;
		
		for (std::string & value : splitValues)
		{
			if (value.empty())
				continue;
			
			const float floatValue = Parse::Float(value);
			
			floatValues.push_back(floatValue);
		}
		
		logDebug("received %d values!", floatValues.size());
		
		//
		
		oscFrame = MotionFrame();
		
		int index = 0;
		
		for (int i = 0; i + 3 <= floatValues.size() && index < MotionFrame::kNumPoints; i += 3, ++index)
		{
			MotionPoint & mp = oscFrame.points[index];
			
			mp.p[0] = floatValues[i + 0] * oscScale;
			mp.p[1] = floatValues[i + 1] * oscScale;
			mp.p[2] = floatValues[i + 2] * oscScale;
			
			oscFrame.numPoints++;
		}
	}
}
