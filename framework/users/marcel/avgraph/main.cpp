#include "Calc.h"
#include "framework.h"
#include "graph.h"
#include "Parse.h"
#include "StringEx.h"
#include "tinyxml2.h"
#include "../avpaint/video.h"

#include "vfxGraph.h"
#include "vfxTypes.h"

#include "vfxNodes/vfxNodeBase.h"
#include "vfxNodes/vfxNodeComposite.h"
#include "vfxNodes/vfxNodeDisplay.h"
#include "vfxNodes/vfxNodeFsfx.h"
#include "vfxNodes/vfxNodeLeapMotion.h"
#include "vfxNodes/vfxNodeLiteral.h"
#include "vfxNodes/vfxNodeLogicSwitch.h"
#include "vfxNodes/vfxNodeMapEase.h"
#include "vfxNodes/vfxNodeMapRange.h"
#include "vfxNodes/vfxNodeMath.h"
#include "vfxNodes/vfxNodeMouse.h"
#include "vfxNodes/vfxNodeNoiseSimplex2D.h"
#include "vfxNodes/vfxNodeOsc.h"
#include "vfxNodes/vfxNodeOscPrimitives.h"
#include "vfxNodes/vfxNodePicture.h"
#include "vfxNodes/vfxNodeSampleAndHold.h"
#include "vfxNodes/vfxNodeTime.h"
#include "vfxNodes/vfxNodeTriggerTimer.h"
#include "vfxNodes/vfxNodeVideo.h"

#include "../libparticle/ui.h"

#include "ccl.h"
#include "cclKinectNode.h"
#include "cclOscNode.h"

using namespace tinyxml2;

/*

todo :
+ replace surface type inputs and outputs to image type
+ add VfxImageBase type. let VfxPlug use this type for image type inputs and outputs. has virtual getTexture method
+ add VfxImage_Surface type. let VfxNodeFsfx use this type
+ add VfxPicture type. type name = 'picture'
+ add VfxImage_Texture type. let VfxPicture use this type
+ add VfxVideo type. type name = 'video'
- add default value to socket definitions
	- add to XML
	- add ability to reset values to their default in UI
+ add editorValue to node inputs and outputs. let get*** methods use this value when plug is not connected
+ let graph editor set editorValue for nodes. only when editor is set on type definition
+ add socket connection selection. remove connection on BACKSPACE
+ add multiple node selection
# on typing 0..9 let node value editor erase editorValue and begin typing. requires state transition? end editing on ENTER or when selecting another entity
# add ability to increment and decrement editorValue. use mouse Y movement or scroll wheel (?)
+ remember number of digits entered after '.' when editing editorValue. use this information when incrementing/decrementing values
- add zoom in/out
	+ add basic implementation
	- improve zoom in and out behavior
		- clamp max zoom level
		- improve font rendering so it's both resolution independent and supports sub-pixel translation
	+ save/load zoom and focus position to/from XML
	+ add option to quickly reset drag and zoom values
	+ use arrow keys to navigate workspace (when no nodes are selected)
+ add sine, saw, triangle and square oscillators
+ save/load link ids
+ save/load next alloc ids for nodes and links
+ free literal values on graph free
+ recreate DatGui when loading graph / current node gets freed
# prioritize input between DatGui and graph editor. do hit test on DatGui
+ add 'color' type name
+ implement OSC node
+ implement Leap Motion node
- add undo/redo support. just serialize/deserialize graph for every action?
	- note : serialize/deserialize entire graph doesn't work nicely with real-time connection
			 we will need to serialize node on remove and re-add/restore it during undo (also invoking real-time connection)
			 same for links and all other actions. we need to perform the opposite action on undo
+ UI element focus: graph editor vs property editor
+ add ability to collapse nodes, so they take up less space
	+ SPACE to toggle
	+ fix hit test
	+ fix link end point locations
+ passthrough toggle on selection: check if all passthrough. yes? disable passthrough, else enable
+ add socket output value editing, for node types that define it on their outputs. required for literals
- add enum value types. use combo box to select values
	- define enums for ease node type
- add ability to randomize input values
# fix white screen issue on Windows when GUI is visible
+ add trigger support
+ add real-time connection
	+ editing values updates values in live version
	+ marking nodes passthrough gets reflected in live
	# disabling nodes and links gets reflected in live -> deferred until later
	+ adding nodes should add nodes in live
	+ removing nodes should remove nodes in live
	+ adding links should add nodes in live
	+ removing links should remove nodes in live
+ add reverse real-time connection
	+ let graph edit sample socket input and output values
		+ let graph edit show a graph of the values when hovering over a socket
+ make it possible to disable nodes
+ make it possible to disable links
- add drag and drop support string literals
+ integrate with UI from libparticle. it supports enums, better color picking, incrementing values up and down in checkboxes
+ add mouse up/down movement support to increment/decrement values of int/float text boxes
+ add option to specify (in UiState) how far text boxes indent their text fields
+ add history of last nodes added
- insert node on pressing enter in the node type name box
	+ or when pressing one of the suggestion buttons
- add suggestion based purely on matching first part of string (no fuzzy string comparison)
	- order of listing should be : pure matches, fuzzy matches, history. show history once type name text box is made active
	- clear type name text box when adding node
- automatically hide UI when mouse/keyboard is inactive for a while
+ remove 'editor' code
+ allocate literal values for unconnected plugs when live-editing change comes in for input
+ show which nodes and links are actively traversed. add live-connection callback to query activity
	+ add support for one-shot activity
	+ add support for continuous activity
+ show min/max on valueplotter
+ add editor options menu
- improve OSC node
	# purchase and evaluate TouchOSC
	- purchase and evaluate Lemur (by Liine)
	- figure out how to best interop with this software
	- adapt OSC node to fit these products
+ save/load editor options to editor XML
+ add editorName to nodes and add a text box to edit it
+ add 2 texture inputs to fsfx node

todo : nodes :
+ add ease node
	+ value
	+ ease type
	+ ease param1
	+ ease param2
	+ mirror?
	+ result
+ add time node
+ add timer node
- add sample.float node
- add sample.image node. outputs r/g/b/a. specify normalized vs screen coords?
+ add impulse response node. measure input impulse response with oscilator at given frequency
+ add sample and hold node. has trigger for input
- add doValuePlotter to ui framework
+ add simplex noise node
+ add binary counter node, outputting 4-8 bit values (1.f or 0.f)
+ add delay node. 4 inputs for delay. take max for delay buffer. delay buffer filled at say fixed 120 hz. 4 outputs delayed values
- add quantize node

todo : fsfx :
- let FSFX use fsfx.vs vertex shader. don't require effects to have their own vertex shader
- expose uniforms/inputs from FSFX pixel shader
- iterate FSFX pixel shaders and generate type definitions based on FSFX name and exposed uniforms

todo : framework :
- optimize text rendering. use a dynamic texture atlas instead of one separate texture for each glyph. drawText should only emit a single draw call

reference :
+ http://www.dsperados.com (company based in Utrecht ? send to Stijn)

*/

#define FILENAME "ccl.xml"

extern const int GFX_SX;
extern const int GFX_SY;

const int GFX_SX = 1024;
const int GFX_SY = 768;

extern void testDatGui();
extern void testNanovg();
extern void testFourier2d();
extern void testChaosGame();
extern void testImpulseResponseMeasurement();

struct VfxNodeTriggerAsFloat : VfxNodeBase
{
	enum Input
	{
		kInput_Trigger,
		kInput_COUNT
	};
	
	enum Output
	{
		kOutput_Value,
		kOutput_COUNT
	};
	
	float outputValue;
	
	VfxNodeTriggerAsFloat()
		: VfxNodeBase()
		, outputValue(0.f)
	{
		resizeSockets(kInput_COUNT, kOutput_COUNT);
		addInput(kInput_Trigger, kVfxPlugType_Trigger);
		addOutput(kOutput_Value, kVfxPlugType_Float, &outputValue);
	}
	
	virtual void tick(const float dt) override
	{
		outputValue = getInputFloat(kInput_Trigger, 0.f);
	}
};

struct VfxNodeBinaryOutput : VfxNodeBase
{
	enum Input
	{
		kInput_Value,
		kInput_Modulo,
		kInput_COUNT
	};
	
	enum Output
	{
		kOutput_Value1,
		kOutput_Value2,
		kOutput_Value3,
		kOutput_Value4,
		kOutput_Value5,
		kOutput_Value6,
		kOutput_COUNT
	};
	
	float outputValue[6];
	
	VfxNodeBinaryOutput()
		: VfxNodeBase()
		, outputValue()
	{
		resizeSockets(kInput_COUNT, kOutput_COUNT);
		addInput(kInput_Value, kVfxPlugType_Float);
		addInput(kInput_Modulo, kVfxPlugType_Int);
		addOutput(kOutput_Value1, kVfxPlugType_Float, &outputValue[0]);
		addOutput(kOutput_Value2, kVfxPlugType_Float, &outputValue[1]);
		addOutput(kOutput_Value3, kVfxPlugType_Float, &outputValue[2]);
		addOutput(kOutput_Value4, kVfxPlugType_Float, &outputValue[3]);
		addOutput(kOutput_Value5, kVfxPlugType_Float, &outputValue[4]);
		addOutput(kOutput_Value6, kVfxPlugType_Float, &outputValue[5]);
	}
	
	virtual void tick(const float dt) override
	{
		double valueAsDouble = getInputFloat(kInput_Value, 0.f);
		
		valueAsDouble = std::round(valueAsDouble);
		
		int valueAsInt = int(valueAsDouble);
		
		const int modulo = getInputInt(kInput_Modulo, 0);
		
		if (modulo != 0)
			valueAsInt %= modulo;
		
		outputValue[0] = (valueAsInt & (1 << 0)) >> 0;
		outputValue[1] = (valueAsInt & (1 << 1)) >> 1;
		outputValue[2] = (valueAsInt & (1 << 2)) >> 2;
		outputValue[3] = (valueAsInt & (1 << 3)) >> 3;
		outputValue[4] = (valueAsInt & (1 << 4)) >> 4;
		outputValue[5] = (valueAsInt & (1 << 5)) >> 5;
	}
};

struct VfxNodeTransform2d : VfxNodeBase
{
	enum Input
	{
		kInput_X,
		kInput_Y,
		kInput_Scale,
		kInput_ScaleX,
		kInput_ScaleY,
		kInput_Angle,
		kInput_COUNT
	};
	
	enum Output
	{
		kOutput_Transform,
		kOutput_COUNT
	};
	
	VfxTransform transform;
	
	VfxNodeTransform2d()
		: VfxNodeBase()
		, transform()
	{
		resizeSockets(kInput_COUNT, kOutput_COUNT);
		addInput(kInput_X, kVfxPlugType_Float);
		addInput(kInput_Y, kVfxPlugType_Float);
		addInput(kInput_Scale, kVfxPlugType_Float);
		addInput(kInput_ScaleX, kVfxPlugType_Float);
		addInput(kInput_ScaleY, kVfxPlugType_Float);
		addInput(kInput_Angle, kVfxPlugType_Float);
		addOutput(kOutput_Transform, kVfxPlugType_Transform, &transform);
	}
	
	virtual void initSelf(const GraphNode & node) override
	{
		// todo : parse node.editorValue;
	}
	
	virtual void tick(const float dt) override
	{
		const float x = getInputFloat(kInput_X, 0.f);
		const float y = getInputFloat(kInput_Y, 0.f);
		const float scale = getInputFloat(kInput_Scale, 1.f);
		const float scaleX = getInputFloat(kInput_ScaleX, 1.f);
		const float scaleY = getInputFloat(kInput_ScaleY, 1.f);
		const float angle = getInputFloat(kInput_Angle, 0.f);
		
		Mat4x4 t;
		Mat4x4 s;
		Mat4x4 r;
		
		t.MakeTranslation(x, y, 0.f);
		s.MakeScaling(scale * scaleX, scale * scaleY, 1.f);
		r.MakeRotationZ(Calc::DegToRad(angle));
		
		transform.matrix = t * r * s;
	}
};

struct VfxNodeDelayLine : VfxNodeBase
{
	const static int kSampleRate = 200;
	
	enum Input
	{
		kInput_Value,
		kInput_Delay1,
		kInput_Delay2,
		kInput_Delay3,
		kInput_Delay4,
		kInput_COUNT
	};
	
	enum Output
	{
		kOutput_Value1,
		kOutput_Value2,
		kOutput_Value3,
		kOutput_Value4,
		kOutput_COUNT
	};
	
	float outputValue[4];
	
	float dtRemaining;
	
	DelayLine delayLine;
	
	VfxNodeDelayLine()
		: VfxNodeBase()
		, outputValue()
		, dtRemaining(0.f)
		, delayLine()
	{
		resizeSockets(kInput_COUNT, kOutput_COUNT);
		addInput(kInput_Value, kVfxPlugType_Float);
		addInput(kInput_Delay1, kVfxPlugType_Float);
		addInput(kInput_Delay2, kVfxPlugType_Float);
		addInput(kInput_Delay3, kVfxPlugType_Float);
		addInput(kInput_Delay4, kVfxPlugType_Float);
		addOutput(kOutput_Value1, kVfxPlugType_Float, &outputValue[0]);
		addOutput(kOutput_Value2, kVfxPlugType_Float, &outputValue[1]);
		addOutput(kOutput_Value3, kVfxPlugType_Float, &outputValue[2]);
		addOutput(kOutput_Value4, kVfxPlugType_Float, &outputValue[3]);
	}
	
	virtual void tick(const float dt) override
	{
		const float delay1 = getInputFloat(kInput_Delay1, 0.f);
		const float delay2 = getInputFloat(kInput_Delay2, 0.f);
		const float delay3 = getInputFloat(kInput_Delay3, 0.f);
		const float delay4 = getInputFloat(kInput_Delay4, 0.f);
	
		const float maxDelay = std::max(std::max(delay1, delay2), std::max(delay3, delay4));
		
		{
			// set delay line length
			
			const int numSamples = maxDelay * kSampleRate;
			
			if (numSamples != delayLine.getLength())
			{
				delayLine.setLength(numSamples);
			}
		}
		
		if (delayLine.getLength() > 0)
		{
			const float value = getInputFloat(kInput_Value, 0.f);
			
			const float dtTotal = dtRemaining + dt;
			
			const int numSamples = std::floor(dtTotal * kSampleRate);
			
			dtRemaining = dtTotal - numSamples / float(kSampleRate);
			
			for (int i = 0; i < numSamples; ++i)
				delayLine.push(value);
			
			const int offset1 = std::min(delayLine.getLength() - 1, int((maxDelay - delay1) * kSampleRate));
			const int offset2 = std::min(delayLine.getLength() - 1, int((maxDelay - delay2) * kSampleRate));
			const int offset3 = std::min(delayLine.getLength() - 1, int((maxDelay - delay3) * kSampleRate));
			const int offset4 = std::min(delayLine.getLength() - 1, int((maxDelay - delay4) * kSampleRate));
			
			outputValue[0] = delayLine.read(offset1);
			outputValue[1] = delayLine.read(offset2);
			outputValue[2] = delayLine.read(offset3);
			outputValue[3] = delayLine.read(offset4);
		}
		else
		{
			outputValue[0] = 0.f;
			outputValue[1] = 0.f;
			outputValue[2] = 0.f;
			outputValue[3] = 0.f;
		}
	}
};

struct VfxNodeImpulseResponse : VfxNodeBase
{
	const static int kSampleRate = 200;
	
	enum Input
	{
		kInput_Value,
		kInput_Frequency,
		kInput_COUNT
	};
	
	enum Output
	{
		kOutput_ImpulseResponse,
		kOutput_COUNT
	};
	
	float impulseResponse;
	
	float dtRemaining;
	
	DelayLine delayLine;
	
	VfxNodeImpulseResponse()
		: VfxNodeBase()
		, impulseResponse(0.f)
		, dtRemaining(0.f)
		, delayLine()
	{
		resizeSockets(kInput_COUNT, kOutput_COUNT);
		addInput(kInput_Value, kVfxPlugType_Float);
		addInput(kInput_Frequency, kVfxPlugType_Float);
		addOutput(kOutput_ImpulseResponse, kVfxPlugType_Float, &impulseResponse);
	}
	
	virtual void tick(const float dt) override
	{
		const float value = getInputFloat(kInput_Value, 0.f);
		const float frequency = getInputFloat(kInput_Frequency, 0.f);
		
		if (frequency <= 0.f)
		{
			delayLine.setLength(0);
		}
		else
		{
			// set delay line length
			
			//const float delay = 1.f / frequency * 4.f;
			const float delay = 1.f / frequency * 2.f;
			
			const int numSamples = delay * kSampleRate;
			
			if (numSamples != delayLine.getLength())
			{
				delayLine.setLength(numSamples);
			}
		}
		
		if (delayLine.getLength() > 0)
		{
			const float dtTotal = dtRemaining + dt;
			
			const int numSamples = std::floor(dtTotal * kSampleRate);
			
			dtRemaining = dtTotal - numSamples / float(kSampleRate);
			
			for (int i = 0; i < numSamples; ++i)
				delayLine.push(value);
		}
		
		//
		
		if (delayLine.getLength() > 0)
		{
			const double twoPi = 2.0 * M_PI;
			
			double measurementPhase = 0.0;
			double measurementStep = twoPi * frequency / kSampleRate;
			
			double sumX = 0.0;
			double sumY = 0.0;
			
			for (int i = 0; i < delayLine.getLength(); ++i)
			{
				const double value = delayLine.read(i);
				
				const double x = std::cos(measurementPhase);
				const double y = std::sin(measurementPhase);
				
				sumX += value * x;
				sumY += value * y;
				
				measurementPhase = std::fmod(measurementPhase + measurementStep, twoPi);
			}
			
			const double avgX = sumX / delayLine.getLength();
			const double avgY = sumY / delayLine.getLength();
			
			impulseResponse = float(std::hypot(avgX, avgY) * 2.0);
		}
	}
};

static VfxNodeBase * createVfxNode(const GraphNodeId nodeId, const std::string & typeName, VfxGraph * vfxGraph)
{
	VfxNodeBase * vfxNode = nullptr;
	
#define DefineNodeImpl(_typeName, _type) \
	else if (typeName == _typeName) \
		vfxNode = new _type();
	
	if (typeName == "intBool")
	{
		vfxNode = new VfxNodeBoolLiteral();
	}
	else if (typeName == "intLiteral")
	{
		vfxNode = new VfxNodeIntLiteral();
	}
	else if (typeName == "floatLiteral")
	{
		vfxNode = new VfxNodeFloatLiteral();
	}
	else if (typeName == "transformLiteral")
	{
		vfxNode = new VfxNodeTransformLiteral();
	}
	else if (typeName == "stringLiteral")
	{
		vfxNode = new VfxNodeStringLiteral();
	}
	else if (typeName == "colorLiteral")
	{
		vfxNode = new VfxNodeColorLiteral();
	}
	DefineNodeImpl("ccl", VfxNodeCCL)
	DefineNodeImpl("ccl.osc", VfxNodeCclOsc)
	DefineNodeImpl("ccl.kinect", VfxNodeCclKinect)
	DefineNodeImpl("trigger.asFloat", VfxNodeTriggerAsFloat)
	DefineNodeImpl("time", VfxNodeTime)
	DefineNodeImpl("sampleAndHold", VfxNodeSampleAndHold)
	DefineNodeImpl("binary.output", VfxNodeBinaryOutput)
	DefineNodeImpl("transform.2d", VfxNodeTransform2d)
	DefineNodeImpl("trigger.timer", VfxNodeTriggerTimer)
	DefineNodeImpl("logic.switch", VfxNodeLogicSwitch)
	DefineNodeImpl("noise.simplex2d", VfxNodeNoiseSimplex2D)
	DefineNodeImpl("sample.delay", VfxNodeDelayLine)
	DefineNodeImpl("impulse.response", VfxNodeImpulseResponse)
	DefineNodeImpl("math.range", VfxNodeMapRange)
	DefineNodeImpl("ease", VfxNodeMapEase)
	DefineNodeImpl("math.add", VfxNodeMathAdd)
	DefineNodeImpl("math.sub", VfxNodeMathSub)
	DefineNodeImpl("math.mul", VfxNodeMathMul)
	DefineNodeImpl("math.sin", VfxNodeMathSin)
	DefineNodeImpl("math.cos", VfxNodeMathCos)
	DefineNodeImpl("math.abs", VfxNodeMathAbs)
	DefineNodeImpl("math.min", VfxNodeMathMin)
	DefineNodeImpl("math.max", VfxNodeMathMax)
	DefineNodeImpl("math.sat", VfxNodeMathSat)
	DefineNodeImpl("math.neg", VfxNodeMathNeg)
	DefineNodeImpl("math.sqrt", VfxNodeMathSqrt)
	DefineNodeImpl("math.pow", VfxNodeMathPow)
	DefineNodeImpl("math.exp", VfxNodeMathExp)
	DefineNodeImpl("math.mod", VfxNodeMathMod)
	DefineNodeImpl("math.fract", VfxNodeMathFract)
	DefineNodeImpl("math.floor", VfxNodeMathFloor)
	DefineNodeImpl("math.ceil", VfxNodeMathCeil)
	DefineNodeImpl("math.round", VfxNodeMathRound)
	DefineNodeImpl("math.sign", VfxNodeMathSign)
	DefineNodeImpl("math.hypot", VfxNodeMathHypot)
	DefineNodeImpl("osc.sine", VfxNodeOscSine)
	DefineNodeImpl("osc.saw", VfxNodeOscSaw)
	DefineNodeImpl("osc.triangle", VfxNodeOscTriangle)
	DefineNodeImpl("osc.square", VfxNodeOscSquare)
	else if (typeName == "display")
	{
		vfxNode = new VfxNodeDisplay();
		
		// fixme : move display node id handling out of here. remove nodeId and vfxGraph passed in to this function
		Assert(vfxGraph->displayNodeId == kGraphNodeIdInvalid);
		vfxGraph->displayNodeId = nodeId;
	}
	else if (typeName == "mouse")
	{
		vfxNode = new VfxNodeMouse();
	}
	else if (typeName == "leap")
	{
		vfxNode = new VfxNodeLeapMotion();
	}
	else if (typeName == "osc")
	{
		vfxNode = new VfxNodeOsc();
	}
	else if (typeName == "composite")
	{
		vfxNode = new VfxNodeComposite();
	}
	else if (typeName == "picture")
	{
		vfxNode = new VfxNodePicture();
	}
	else if (typeName == "video")
	{
		vfxNode = new VfxNodeVideo();
	}
	else if (typeName == "fsfx")
	{
		vfxNode = new VfxNodeFsfx();
	}
	else
	{
		logError("unknown node type: %s", typeName.c_str());
	}
	
#undef DefineNodeImpl

	return vfxNode;
}

static VfxGraph * constructVfxGraph(const Graph & graph, const GraphEdit_TypeDefinitionLibrary * typeDefinitionLibrary)
{
	VfxGraph * vfxGraph = new VfxGraph();
	
	for (auto nodeItr : graph.nodes)
	{
		auto & node = nodeItr.second;
		
		if (node.isEnabled == false)
		{
			continue;
		}
		
		VfxNodeBase * vfxNode = createVfxNode(node.id, node.typeName, vfxGraph);
		
		Assert(vfxNode != nullptr);
		if (vfxNode == nullptr)
		{
			logError("unable to create node");
		}
		else
		{
			vfxNode->isPassthrough = node.editorIsPassthrough;
			
			vfxNode->initSelf(node);
			
			vfxGraph->nodes[node.id] = vfxNode;
		}
	}
	
	for (auto & linkItr : graph.links)
	{
		auto & link = linkItr.second;
		
		if (link.isEnabled == false)
		{
			continue;
		}
		
		auto srcNodeItr = vfxGraph->nodes.find(link.srcNodeId);
		auto dstNodeItr = vfxGraph->nodes.find(link.dstNodeId);
		
		Assert(srcNodeItr != vfxGraph->nodes.end() && dstNodeItr != vfxGraph->nodes.end());
		if (srcNodeItr == vfxGraph->nodes.end() || dstNodeItr == vfxGraph->nodes.end())
		{
			if (srcNodeItr == vfxGraph->nodes.end())
				logError("source node doesn't exist");
			if (dstNodeItr == vfxGraph->nodes.end())
				logError("destination node doesn't exist");
		}
		else
		{
			auto srcNode = srcNodeItr->second;
			auto dstNode = dstNodeItr->second;
			
			auto input = srcNode->tryGetInput(link.srcNodeSocketIndex);
			auto output = dstNode->tryGetOutput(link.dstNodeSocketIndex);
			
			Assert(input != nullptr && output != nullptr);
			if (input == nullptr || output == nullptr)
			{
				if (input == nullptr)
					logError("input node socket doesn't exist");
				if (output == nullptr)
					logError("output node socket doesn't exist");
			}
			else
			{
				input->connectTo(*output);
				
				// note : this may add the same node multiple times to the list of predeps. note that this
				//        is ok as nodes will be traversed once through the travel id + it works nicely
				//        with the live connection as we can just remove the predep and still have one or
				//        references to the predep if the predep was referenced more than once
				srcNode->predeps.push_back(dstNode);
				
				// if this is a trigger, add a trigger target to dstNode
				if (output->type == kVfxPlugType_Trigger)
				{
					VfxNodeBase::TriggerTarget triggerTarget;
					triggerTarget.srcNode = srcNode;
					triggerTarget.srcSocketIndex = link.srcNodeSocketIndex;
					triggerTarget.dstSocketIndex = link.dstNodeSocketIndex;
					
					dstNode->triggerTargets.push_back(triggerTarget);
				}
			}
		}
	}
	
	for (auto nodeItr : graph.nodes)
	{
		auto & node = nodeItr.second;
		
		auto typeDefintion = typeDefinitionLibrary->tryGetTypeDefinition(node.typeName);
		
		if (typeDefintion == nullptr)
			continue;
		
		auto vfxNodeItr = vfxGraph->nodes.find(node.id);
		
		if (vfxNodeItr == vfxGraph->nodes.end())
			continue;
		
		VfxNodeBase * vfxNode = vfxNodeItr->second;
		
		auto & vfxNodeInputs = vfxNode->inputs;
		
		for (auto inputValueItr : node.editorInputValues)
		{
			const std::string & inputName = inputValueItr.first;
			const std::string & inputValue = inputValueItr.second;
			
			for (size_t i = 0; i < typeDefintion->inputSockets.size(); ++i)
			{
				if (typeDefintion->inputSockets[i].name == inputName)
				{
					if (i < vfxNodeInputs.size())
					{
						if (vfxNodeInputs[i].isConnected() == false)
						{
							vfxGraph->connectToInputLiteral(vfxNodeInputs[i], inputValue);
						}
					}
				}
			}
		}
	}
	
	for (auto vfxNodeItr : vfxGraph->nodes)
	{
		auto nodeId = vfxNodeItr.first;
		auto nodeItr = graph.nodes.find(nodeId);
		auto & node = nodeItr->second;
		auto vfxNode = vfxNodeItr.second;
		
		vfxNode->init(node);
	}
	
	return vfxGraph;
}

struct RealTimeConnection : GraphEdit_RealTimeConnection
{
	VfxGraph * vfxGraph;
	VfxGraph ** vfxGraphPtr;
	
	bool isLoading;
	
	RealTimeConnection()
		: GraphEdit_RealTimeConnection()
		, vfxGraph(nullptr)
		, vfxGraphPtr(nullptr)
		, isLoading(false)
	{
	}
	
	virtual void loadBegin() override
	{
		isLoading = true;
		
		delete vfxGraph;
		vfxGraph = nullptr;
		*vfxGraphPtr = nullptr;
	}
	
	virtual void loadEnd(GraphEdit & graphEdit) override
	{
		vfxGraph = constructVfxGraph(*graphEdit.graph, graphEdit.typeDefinitionLibrary);
		*vfxGraphPtr = vfxGraph;
		
		isLoading = false;
	}
	
	virtual void nodeAdd(const GraphNodeId nodeId, const std::string & typeName) override
	{
		if (isLoading)
			return;
		
		logDebug("nodeAdd");
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr == vfxGraph->nodes.end());
		if (nodeItr != vfxGraph->nodes.end())
			return;
		
		//
		
		GraphNode node;
		node.id = nodeId;
		node.typeName = typeName;
		
		//
		
		VfxNodeBase * vfxNode = createVfxNode(nodeId, typeName, vfxGraph);
		
		if (vfxNode == nullptr)
			return;
		
		vfxNode->initSelf(node);
		
		vfxGraph->nodes[node.id] = vfxNode;
		
		//
		
		vfxNode->init(node);
	}
	
	virtual void nodeRemove(const GraphNodeId nodeId) override
	{
		if (isLoading)
			return;
		
		logDebug("nodeRemove");
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr != vfxGraph->nodes.end());
		if (nodeItr == vfxGraph->nodes.end())
			return;
		
		auto node = nodeItr->second;
		
		// all links should be removed at this point. there should be no remaining predeps or connected nodes
		Assert(node->predeps.empty());
		//for (auto & input : node->inputs)
		//	Assert(!input.isConnected()); // may be a literal value node with a non-accounted for (in the graph) connection when created directly from socket value
		// todo : iterate all other nodes, to ensure there are no nodes with references back to this node?
		
		delete node;
		node = nullptr;
		
		vfxGraph->nodes.erase(nodeItr);
	}
	
	virtual void linkAdd(const GraphLinkId linkId, const GraphNodeId srcNodeId, const int srcSocketIndex, const GraphNodeId dstNodeId, const int dstSocketIndex) override
	{
		if (isLoading)
			return;
		
		logDebug("linkAdd");
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return;
		
		auto srcNodeItr = vfxGraph->nodes.find(srcNodeId);
		auto dstNodeItr = vfxGraph->nodes.find(dstNodeId);
		
		Assert(srcNodeItr != vfxGraph->nodes.end() && dstNodeItr != vfxGraph->nodes.end());
		if (srcNodeItr == vfxGraph->nodes.end() || dstNodeItr == vfxGraph->nodes.end())
		{
			if (srcNodeItr == vfxGraph->nodes.end())
				logError("source node doesn't exist");
			if (dstNodeItr == vfxGraph->nodes.end())
				logError("destination node doesn't exist");
			
			return;
		}

		auto srcNode = srcNodeItr->second;
		auto dstNode = dstNodeItr->second;
		
		auto input = srcNode->tryGetInput(srcSocketIndex);
		auto output = dstNode->tryGetOutput(dstSocketIndex);
		
		Assert(input != nullptr && output != nullptr);
		if (input == nullptr || output == nullptr)
		{
			if (input == nullptr)
				logError("input node socket doesn't exist");
			if (output == nullptr)
				logError("output node socket doesn't exist");
			
			return;
		}
		
		input->connectTo(*output);
		
		// note : this may add the same node multiple times to the list of predeps. note that this
		//        is ok as nodes will be traversed once through the travel id + it works nicely
		//        with the live connection as we can just remove the predep and still have one or
		//        references to the predep if the predep was referenced more than once
		srcNode->predeps.push_back(dstNode);
		
		// if this is a trigger, add a trigger target to dstNode
		if (output->type == kVfxPlugType_Trigger)
		{
			VfxNodeBase::TriggerTarget triggerTarget;
			triggerTarget.srcNode = srcNode;
			triggerTarget.srcSocketIndex = srcSocketIndex;
			triggerTarget.dstSocketIndex = dstSocketIndex;
			
			dstNode->triggerTargets.push_back(triggerTarget);
		}
	}
	
	virtual void linkRemove(const GraphLinkId linkId, const GraphNodeId srcNodeId, const int srcSocketIndex, const GraphNodeId dstNodeId, const int dstSocketIndex) override
	{
		if (isLoading)
			return;
		
		logDebug("linkRemove");
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return;
		
		auto srcNodeItr = vfxGraph->nodes.find(srcNodeId);
		
		Assert(srcNodeItr != vfxGraph->nodes.end());
		if (srcNodeItr == vfxGraph->nodes.end())
			return;
		
		auto srcNode = srcNodeItr->second;
		
		auto input = srcNode->tryGetInput(srcSocketIndex);
		
		Assert(input != nullptr);
		if (input == nullptr)
			return;
		
		Assert(input->isConnected());
		input->disconnect();
		
		{
			// attempt to remove dst node from predeps
			
			auto dstNodeItr = vfxGraph->nodes.find(dstNodeId);
			
			Assert(dstNodeItr != vfxGraph->nodes.end());
			if (dstNodeItr != vfxGraph->nodes.end())
			{
				auto dstNode = dstNodeItr->second;
				
				bool foundPredep = false;
				
				for (auto i = srcNode->predeps.begin(); i != srcNode->predeps.end(); ++i)
				{
					VfxNodeBase * vfxNode = *i;
					
					if (vfxNode == dstNode)
					{
						srcNode->predeps.erase(i);
						foundPredep = true;
						break;
					}
				}
				
				Assert(foundPredep);
			}
		}
		
		// if this is a link hooked up to a trigger, remove the TriggerTarget from dstNode
		
		if (input->type == kVfxPlugType_Trigger)
		{
			auto dstNodeItr = vfxGraph->nodes.find(dstNodeId);
			
			Assert(dstNodeItr != vfxGraph->nodes.end());
			if (dstNodeItr != vfxGraph->nodes.end())
			{
				auto dstNode = dstNodeItr->second;
				
				bool foundTriggerTarget = false;
				
				for (auto triggerTargetItr = dstNode->triggerTargets.begin(); triggerTargetItr != dstNode->triggerTargets.end(); ++triggerTargetItr)
				{
					auto & triggerTarget = *triggerTargetItr;
					
					if (triggerTarget.srcNode == srcNode && triggerTarget.srcSocketIndex == srcSocketIndex)
					{
						dstNode->triggerTargets.erase(triggerTargetItr);
						foundTriggerTarget = true;
						break;
					}
				}
				
				Assert(foundTriggerTarget);
			}
		}
	}
	
	virtual void setNodeIsPassthrough(const GraphNodeId nodeId, const bool isPassthrough) override
	{
		if (isLoading)
			return;
		
		logDebug("setNodeIsPassthrough called for nodeId=%d, isPassthrough=%d", int(nodeId), int(isPassthrough));
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr != vfxGraph->nodes.end());
		if (nodeItr == vfxGraph->nodes.end())
			return;
		
		auto node = nodeItr->second;
		
		node->isPassthrough = isPassthrough;
	}
	
	static bool setPlugValue(VfxPlug * plug, const std::string & value)
	{
		switch (plug->type)
		{
		case kVfxPlugType_None:
			return false;
		case kVfxPlugType_Bool:
			plug->getRwBool() = Parse::Bool(value);
			return true;
		case kVfxPlugType_Int:
			plug->getRwInt() = Parse::Int32(value);
			return true;
		case kVfxPlugType_Float:
			plug->getRwFloat() = Parse::Float(value);
			return true;
			
		case kVfxPlugType_Transform:
			return false;
			
		case kVfxPlugType_String:
			plug->getRwString() = value;
			return true;
		case kVfxPlugType_Color:
			plug->getRwColor() = Color::fromHex(value.c_str());
			return true;
			
		case kVfxPlugType_Image:
			return false;
		case kVfxPlugType_Surface:
			return false;
		case kVfxPlugType_Trigger:
			return false;
		}
		
		Assert(false); // all cases should be handled explicitly
		return false;
	}
	
	virtual void setSrcSocketValue(const GraphNodeId nodeId, const int srcSocketIndex, const std::string & srcSocketName, const std::string & value) override
	{
		if (isLoading)
			return;
		
		logDebug("setSrcSocketValue called for nodeId=%d, srcSocket=%s", int(nodeId), srcSocketName.c_str());
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr != vfxGraph->nodes.end());
		if (nodeItr == vfxGraph->nodes.end())
			return;
		
		auto node = nodeItr->second;
		
		auto input = node->tryGetInput(srcSocketIndex);
		
		Assert(input != nullptr);
		if (input == nullptr)
			return;
		
		if (input->isConnected())
		{
			setPlugValue(input, value);
		}
		else
		{
			vfxGraph->connectToInputLiteral(*input, value);
		}
	}
	
	static bool getPlugValue(VfxPlug * plug, std::string & value)
	{
		switch (plug->type)
		{
		case kVfxPlugType_None:
			return false;
		case kVfxPlugType_Bool:
			value = String::ToString(plug->getBool());
			return true;
		case kVfxPlugType_Int:
			value = String::ToString(plug->getInt());
			return true;
		case kVfxPlugType_Float:
			value = String::FormatC("%f", plug->getFloat());
			return true;
		case kVfxPlugType_Transform:
			return false;
		case kVfxPlugType_String:
			value = plug->getString();
			return true;
		case kVfxPlugType_Color:
			{
				const Color & color = plug->getColor();
				value = color.toHexString(true);
				return true;
			}
		case kVfxPlugType_Image:
			value = String::ToString(plug->getImage()->getTexture());
			return true;
		case kVfxPlugType_Surface:
			return false;
		case kVfxPlugType_Trigger:
			{
				const VfxTriggerData & triggerData = plug->getTriggerData();
				
				if (triggerData.type == kVfxTriggerDataType_Bool ||
					triggerData.type == kVfxTriggerDataType_Int)
				{
					value = String::FormatC("%d", triggerData.asInt());
					return true;
				}
				else if (triggerData.type == kVfxTriggerDataType_Float)
				{
					value = String::FormatC("%f", triggerData.asFloat());
					return true;
				}
				else
				{
					Assert(false);
					return false;
				}
			}
			return false;
		}
		
		Assert(false); // all cases should be handled explicitly
		return false;
	}
	
	virtual bool getSrcSocketValue(const GraphNodeId nodeId, const int srcSocketIndex, const std::string & srcSocketName, std::string & value) override
	{
		if (isLoading)
			return false;
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return false;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr != vfxGraph->nodes.end());
		if (nodeItr == vfxGraph->nodes.end())
			return false;
		
		auto node = nodeItr->second;
		
		auto input = node->tryGetInput(srcSocketIndex);
		
		Assert(input != nullptr);
		if (input == nullptr)
			return false;
		
		if (input->isConnected() == false)
			return false;
		
		return getPlugValue(input, value);
	}
	
	virtual void setDstSocketValue(const GraphNodeId nodeId, const int dstSocketIndex, const std::string & dstSocketName, const std::string & value) override
	{
		if (isLoading)
			return;
		
		logDebug("setDstSocketValue called for nodeId=%d, dstSocket=%s", int(nodeId), dstSocketName.c_str());
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr != vfxGraph->nodes.end());
		if (nodeItr == vfxGraph->nodes.end())
			return;
		
		auto node = nodeItr->second;
		
		auto output = node->tryGetOutput(dstSocketIndex);
		
		Assert(output != nullptr);
		if (output == nullptr)
			return;
		
		setPlugValue(output, value);
	}
	
	virtual bool getDstSocketValue(const GraphNodeId nodeId, const int dstSocketIndex, const std::string & dstSocketName, std::string & value) override
	{
		if (isLoading)
			return false;
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return false;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr != vfxGraph->nodes.end());
		if (nodeItr == vfxGraph->nodes.end())
			return false;
		
		auto node = nodeItr->second;
		
		auto output = node->tryGetOutput(dstSocketIndex);
		
		Assert(output != nullptr);
		if (output == nullptr)
			return false;
		
		return getPlugValue(output, value);
	}
	
	virtual int nodeIsActive(const GraphNodeId nodeId) override
	{
		if (isLoading)
			return false;
		
		Assert(vfxGraph != nullptr);
		if (vfxGraph == nullptr)
			return false;
		
		auto nodeItr = vfxGraph->nodes.find(nodeId);
		
		Assert(nodeItr != vfxGraph->nodes.end());
		if (nodeItr == vfxGraph->nodes.end())
			return false;
		
		auto node = nodeItr->second;
		
		int result = 0;
		
		if (node->lastDrawTraversalId + 1 == vfxGraph->nextDrawTraversalId)
			result |= kActivity_Continuous;
		
		if (node->editorIsTriggered)
		{
			result |= kActivity_OneShot;
			
			node->editorIsTriggered = false;
		}
		
		return result;
	}
	
	virtual int linkIsActive(const GraphLinkId linkId) override
	{
		if (isLoading)
			return false;
		
		return false;
	}
};

int main(int argc, char * argv[])
{
	//framework.waitForEvents = true;
	
	framework.enableRealTimeEditing = true;
	
	//framework.minification = 2;
	
	framework.enableDepthBuffer = true;
	
	if (framework.init(0, nullptr, GFX_SX, GFX_SY))
	{
		initUi();
		
		//testDatGui();
		
		//testNanovg();
		
		//testChaosGame();
		
		//testFourier2d();
		
		//testImpulseResponseMeasurement();
		
		//
		
		GraphEdit_TypeDefinitionLibrary * typeDefinitionLibrary = new GraphEdit_TypeDefinitionLibrary();
		
		{
			XMLDocument * document = new XMLDocument();
			
			if (document->LoadFile("types.xml") == XML_SUCCESS)
			{
				const XMLElement * xmlLibrary = document->FirstChildElement("library");
				
				if (xmlLibrary != nullptr)
				{
					typeDefinitionLibrary->loadXml(xmlLibrary);
				}
			}
			
			delete document;
			document = nullptr;
		}
		
		//
		
		RealTimeConnection * realTimeConnection = new RealTimeConnection();
		
		//
		
		GraphEdit * graphEdit = new GraphEdit(typeDefinitionLibrary);
		
		graphEdit->realTimeConnection = realTimeConnection;

		VfxGraph * vfxGraph = new VfxGraph();
		
		realTimeConnection->vfxGraph = vfxGraph;
		realTimeConnection->vfxGraphPtr = &vfxGraph;
		
		graphEdit->load(FILENAME);
		
		while (!framework.quitRequested)
		{
			framework.process();
			
			if (keyboard.wentDown(SDLK_ESCAPE))
				framework.quitRequested = true;
			
			const float dt = framework.timeStep;
			
			if (graphEdit->tick(dt))
			{
			}
			else if (keyboard.wentDown(SDLK_s))
			{
				graphEdit->save(FILENAME);
			}
			else if (keyboard.wentDown(SDLK_l))
			{
				graphEdit->load(FILENAME);
			}
			
			// fixme : this should be handled by graph edit
			
			if (!graphEdit->selectedNodes.empty())
			{
				const GraphNodeId nodeId = *graphEdit->selectedNodes.begin();
				
				graphEdit->propertyEditor->setNode(nodeId);
			}
			
			framework.beginDraw(31, 31, 31, 255);
			{
				if (vfxGraph != nullptr)
				{
					vfxGraph->tick(framework.timeStep);
					vfxGraph->draw();
				}
				
				graphEdit->draw();
			}
			framework.endDraw();
		}
		
		delete graphEdit;
		graphEdit = nullptr;
		
		delete realTimeConnection;
		realTimeConnection = nullptr;
		
		delete typeDefinitionLibrary;
		typeDefinitionLibrary = nullptr;
		
		shutUi();
		
		framework.shutdown();
	}

	return 0;
}
