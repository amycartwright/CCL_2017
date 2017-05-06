#include "vfxNodeOsc.h"

#include "ip/UdpSocket.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscPacketListener.h"

#include <list>

// todo : use port & ip address from socket inputs

#define OSC_RECV_PORT 7000

struct OscMessage
{
	OscMessage()
		: event()
		, str()
	{
		memset(param, 0, sizeof(param));
	}
	
	std::string event;
	float param[4];
	std::string str;
};

class MyOscPacketListener : public osc::OscPacketListener
{
public:
	SDL_mutex * oscMessageMtx;
	std::list<OscMessage> oscMessages;
	
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

			OscMessage message;

			if (strcmp(m.AddressPattern(), "/event") == 0)
			{
				message.event = m.AddressPattern();
				message.str = std::string(m.AddressPattern()).substr(1);
			}
			else
			{
				logWarning("unknown message type: %s", m.AddressPattern());
			}

			if (!message.event.empty())
			{
				SDL_LockMutex(oscMessageMtx);
				{
					logDebug("enqueue OSC message. event=%s", message.event.c_str());

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

VfxNodeOsc::VfxNodeOsc()
	: VfxNodeBase()
	, eventId()
	, oscPacketListener(nullptr)
	, oscReceiveSocket(nullptr)
	, oscMessageThread(nullptr)
{
	resizeSockets(kInput_COUNT, kOutput_COUNT);
	addInput(kInput_Port, kVfxPlugType_Int);
	addInput(kInput_IpAddress, kVfxPlugType_String);
	addOutput(kOutput_Trigger, kVfxPlugType_Trigger, &eventId);
}

VfxNodeOsc::~VfxNodeOsc()
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

void VfxNodeOsc::init(const GraphNode & node)
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
		}
		
		logDebug("creating OSC receive thread");
		
		oscMessageThread = SDL_CreateThread(executeOscThread, "OSC thread", this);
	}
	catch (std::exception & e)
	{
		logError("failed to start OSC receive thread: %s", e.what());
	}
}

void VfxNodeOsc::tick(const float dt)
{
	// update network input

	SDL_LockMutex(oscPacketListener->oscMessageMtx);
	{
		while (!oscPacketListener->oscMessages.empty())
		{
			const OscMessage message = oscPacketListener->oscMessages.front();
			
			oscPacketListener->oscMessages.pop_front();
			
			SDL_UnlockMutex(oscPacketListener->oscMessageMtx);
			{
				// todo : store OSC values in trigger mem
				
				trigger(kOutput_Trigger);
			}
			SDL_LockMutex(oscPacketListener->oscMessageMtx);
		}
	}
	SDL_UnlockMutex(oscPacketListener->oscMessageMtx);
}

int VfxNodeOsc::executeOscThread(void * data)
{
	VfxNodeOsc * self = (VfxNodeOsc*)data;
	
	self->oscReceiveSocket->Run();
	
	return 0;
}

