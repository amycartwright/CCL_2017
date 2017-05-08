#include "cclOscNode.h"

#include "ip/UdpSocket.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscPacketListener.h"

#include "StringEx.h"

#include <list>

struct CclOscMessage
{
	static const int kNumFloats = 75;
	
	CclOscMessage()
		: event()
		, values()
		, numValues(0)
	{
		memset(values, 0, sizeof(values));
	}
	
	std::string event;
	std::string str;
	float values[kNumFloats];
	int numValues;
};

class MyOscPacketListener : public osc::OscPacketListener
{
public:
	SDL_mutex * oscMessageMtx;
	std::list<CclOscMessage> oscMessages;
	
	MyOscPacketListener()
		: oscMessageMtx(nullptr)
		, oscMessages()
	{
		oscMessageMtx = SDL_CreateMutex();
	}
	
	~MyOscPacketListener()
	{
		SDL_DestroyMutex(oscMessageMtx);
		oscMessageMtx = nullptr;
	}
	
protected:
	virtual void ProcessBundle(const osc::ReceivedBundle & b, const IpEndpointName & remoteEndpoint) override
	{
		//logDebug("ProcessBundle: timeTag=%llu", b.TimeTag());

		osc::OscPacketListener::ProcessBundle(b, remoteEndpoint);
	}

	virtual void ProcessMessage(const osc::ReceivedMessage & m, const IpEndpointName & remoteEndpoint) override
	{
		try
		{
			//logDebug("ProcessMessage");

			osc::ReceivedMessageArgumentStream args = m.ArgumentStream();

			CclOscMessage message;

			//if (strcmp(m.AddressPattern(), "/k2/joints/xyz") == 0)
			if (true)
			{
				message.event = m.AddressPattern();
				message.str = std::string(m.AddressPattern()).substr(1);
				
				for (int i = 0; i < CclOscMessage::kNumFloats; ++i)
					args >> message.values[i];
			}
			else
			{
				logWarning("unknown message type: %s", m.AddressPattern());
			}

			if (!message.event.empty())
			{
				SDL_LockMutex(oscMessageMtx);
				{
					//logDebug("enqueue OSC message. event=%s", message.event.c_str());

					oscMessages.push_back(message);
				}
				SDL_UnlockMutex(oscMessageMtx);
			}
		}
		catch (osc::Exception & e)
		{
			logError("error while parsing message: %s: %s", m.AddressPattern(), e.what());
		}
	}
};

VfxNodeCclOsc::VfxNodeCclOsc()
	: VfxNodeBase()
	, eventId()
	, oscPacketListener(nullptr)
	, oscReceiveSocket(nullptr)
	, oscMessageThread(nullptr)
	, outputValues()
{
	resizeSockets(kInput_COUNT, kOutput_COUNT);
	addInput(kInput_Port, kVfxPlugType_Int);
	addInput(kInput_IpAddress, kVfxPlugType_String);
	addOutput(kOutput_Trigger, kVfxPlugType_Trigger, &eventId);
	addOutput(kOutput_Values, kVfxPlugType_String, &outputValues);
}

VfxNodeCclOsc::~VfxNodeCclOsc()
{
	logDebug("terminating OSC receive thread");
	
	if (oscReceiveSocket != nullptr)
	{
		oscReceiveSocket->AsynchronousBreak();
	}
	
	if (oscMessageThread != nullptr)
	{
		SDL_WaitThread(oscMessageThread, nullptr);
		oscMessageThread = nullptr;
	}
	
	logDebug("terminating OSC receive thread [done]");
	
	logDebug("terminating OSC UDP receive socket");
	
	delete oscReceiveSocket;
	oscReceiveSocket = nullptr;
	
	logDebug("terminating OSC UDP receive socket [done]");
	
	delete oscPacketListener;
	oscPacketListener = nullptr;
}

void VfxNodeCclOsc::init(const GraphNode & node)
{
	try
	{
		// create OSC client and listen
		
		oscPacketListener = new MyOscPacketListener();
		
		const std::string ipAddress = getInputString(kInput_IpAddress, "");
		const int udpPort = getInputInt(kInput_Port, 0);
		
		if (ipAddress.empty() || udpPort == 0)
		{
			logWarning("invalid OSC bind address: %s:%d", ipAddress.c_str(), udpPort);
		}
		else
		{
			logDebug("creating OSC UDP receive socket @ %s:%d", ipAddress.c_str(), udpPort);
			
			// IpEndpointName::ANY_ADDRESS
			
			oscReceiveSocket = new UdpListeningReceiveSocket(IpEndpointName(ipAddress.c_str(), udpPort), oscPacketListener);
			
			logDebug("creating OSC receive thread");
		
			oscMessageThread = SDL_CreateThread(executeOscThread, "OSC thread", this);
		}
	}
	catch (std::exception & e)
	{
		logError("failed to start OSC receive thread: %s", e.what());
	}
}

void VfxNodeCclOsc::tick(const float dt)
{
	// update network input

	SDL_LockMutex(oscPacketListener->oscMessageMtx);
	{
		while (!oscPacketListener->oscMessages.empty())
		{
			const CclOscMessage message = oscPacketListener->oscMessages.front();
			
			oscPacketListener->oscMessages.pop_front();
			
			SDL_UnlockMutex(oscPacketListener->oscMessageMtx);
			{
				// todo : store OSC values in trigger mem
				
				outputValues.clear();
				
				for (int i = 0; i < message.kNumFloats; ++i)
				{
					outputValues = outputValues + "," + String::FormatC("%f", message.values[i]);
				}
				
				trigger(kOutput_Trigger);
			}
			SDL_LockMutex(oscPacketListener->oscMessageMtx);
		}
	}
	SDL_UnlockMutex(oscPacketListener->oscMessageMtx);
}

int VfxNodeCclOsc::executeOscThread(void * data)
{
	VfxNodeCclOsc * self = (VfxNodeCclOsc*)data;
	
	self->oscReceiveSocket->Run();
	
	return 0;
}
