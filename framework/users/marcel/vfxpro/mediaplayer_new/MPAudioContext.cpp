#include "MPAudioBuffer.h"
#include "MPAudioContext.h"
#include "MPContext.h"
#include "MPDebug.h"
#include "MPPacketQueue.h"
#include "MPUtil.h"
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswresample/swresample.h>

#define QUEUE_SIZE (4 * 3 * 10)

namespace MP
{
	AudioContext::AudioContext()
		: m_packetQueue(nullptr)
		, m_audioBuffer(nullptr)
		, m_codecContext(nullptr)
		, m_codec(nullptr)
		, m_swrContext(nullptr)
		, m_streamIndex(-1)
		, m_time(0.0)
		, m_frameTime(0)
		, m_initialized(false)
	{
	}

	AudioContext::~AudioContext()
	{
		Assert(m_initialized == false);
	}

	bool AudioContext::Initialize(Context * context, const size_t streamIndex)
	{
		Assert(m_initialized == false);

		m_initialized = true;

		m_streamIndex = streamIndex;
		m_time = 0.0;
		m_frameTime = 0;
		
		Assert(m_packetQueue == nullptr);
		m_packetQueue = new PacketQueue();
		
		Assert(m_audioBuffer == nullptr);
		m_audioBuffer = new AudioBuffer();
		
		AVCodecParameters * audioParams = context->GetFormatContext()->streams[m_streamIndex]->codecpar;\
		if (!audioParams)
		{
			Debug::Print("Audio: failed to find audio params.");
			return false;
		}
		
		// Get codec for audio stream.
		m_codec = avcodec_find_decoder(audioParams->codec_id);
		if (!m_codec)
		{
			Debug::Print("Audio: unable to find codec.");
			return false;
		}
		
		Debug::Print("Audio: codec: %s.", m_codec->name);
		
		// Get codec context for audio stream.
		m_codecContext = avcodec_alloc_context3(m_codec);
		if (!m_codecContext)
		{
			Debug::Print("Audio: failed to allocate codec context.");
			return false;
		}
		
		if (avcodec_parameters_to_context(m_codecContext, audioParams) < 0)
		{
			Debug::Print("Audio: failed to set params on codec context.");
			return false;
		}

		// Open codec.
		if (avcodec_open2(m_codecContext, m_codec, nullptr) < 0)
		{
			Debug::Print("Audio: failed to open codec context.");
			return false;
		}

		// Display codec info.
		Debug::Print("Audio: samplerate: %d.", m_codecContext->sample_rate);
		Debug::Print("Audio: channels: %d.", m_codecContext->channels);
		Debug::Print("Audio: framesize: %d.", m_codecContext->frame_size); // Number of samples/packet.
		
		Assert(m_swrContext == nullptr);
		m_swrContext = swr_alloc_set_opts(nullptr,
			AV_CH_LAYOUT_STEREO, AV_SAMPLE_FMT_S16, m_codecContext->sample_rate,
			audioParams->channel_layout, (AVSampleFormat)audioParams->format, audioParams->sample_rate,
		0, nullptr);
		
		if (!m_swrContext)
		{
			Debug::Print("Audio: failed to alloc/init swr context.");
			return false;
		}
		
		return true;
	}

	bool AudioContext::Destroy()
	{
		Assert(m_initialized == true);

		bool result = true;

		m_initialized = false;

		if (m_swrContext)
		{
			swr_free(&m_swrContext);
			m_swrContext = nullptr;
		}
		
		// Close audio codec context.
		if (m_codecContext)
		{
			avcodec_free_context(&m_codecContext);
			m_codecContext = nullptr;
		}
		
		if (m_codec)
		{
			m_codec = nullptr;
		}
		
		if (m_audioBuffer)
		{
			delete m_audioBuffer;
			m_audioBuffer = nullptr;
		}
		
		if (m_packetQueue)
		{
			delete m_packetQueue;
			m_packetQueue = nullptr;
		}

		return result;
	}

	size_t AudioContext::GetStreamIndex() const
	{
		return m_streamIndex;
	}

	double AudioContext::GetTime() const
	{
		return m_time;
	}

	bool AudioContext::FillAudioBuffer()
	{
		bool result = true;

		while (m_packetQueue->GetSize() > 0)
		{
			Debug::Print("\tAudio: decoding to provide more frames.");
			Debug::Print("\t\tAudio: packet queue size: %d -> %d.", int(m_packetQueue->GetSize()), int(m_packetQueue->GetSize() - 1));

			AVPacket & packet = m_packetQueue->GetPacket();

			if (ProcessPacket(packet) != true)
				result = false;

			m_packetQueue->PopFront();
		}

		return result;
	}

	bool AudioContext::RequestAudio(int16_t * out_samples, const size_t _frameCount, bool & out_gotAudio)
	{
		size_t frameCount = _frameCount;
		
		Assert(out_samples);

		bool result = true;

		out_gotAudio = false;

		bool stop = false;

		Debug::Print("Audio: begin request.");

		while (frameCount > 0 && stop == false)
		{
			size_t numSamples = frameCount * m_codecContext->channels;

			if (!m_audioBuffer->ReadSamples(out_samples, numSamples))
			{
				stop = true;
			}
			else
			{
				const size_t numFrames = numSamples / m_codecContext->channels;

				out_samples += numFrames * m_codecContext->channels;

				frameCount -= numFrames;
				m_frameTime += numFrames;
				m_time = m_frameTime / (double)m_codecContext->sample_rate;

				Debug::Print("\tAudio: read from buffer. time: %03.3f", m_time);

				if (frameCount == 0)
				{
					Debug::Print("\tAudio: read done.");
				}
			}
		}

		Debug::Print("Audio: end request.");

		out_gotAudio = true;

		return result;
	}

	bool AudioContext::IsQueueFull() const
	{
		return m_packetQueue->GetSize() >= QUEUE_SIZE;
	}

	bool AudioContext::AddPacket(AVPacket & packet)
	{
		m_packetQueue->PushBack(packet);

		Debug::Print("PQAUDIO");

		return true;
	}

	bool AudioContext::ProcessPacket(AVPacket & _packet)
	{
		Assert(_packet.data);
		Assert(_packet.size >= 0);

		AVPacket packet = _packet;
		AVFrame * frame = av_frame_alloc();
		
		int bytesRemaining = packet.size;
		
		// Decode packet.
		while (bytesRemaining > 0)
		{
			// Decode data.
			int frameSize = 0;
			
			// todo : decode into segment
			
			AudioBufferSegment segment;
			
			int gotFrame = 0;
			int bytesDecoded = avcodec_decode_audio4(
				m_codecContext,
				frame,
				&gotFrame,
				&packet);

			// Error?
			if (bytesDecoded < 0)
			{
				Debug::Print("Audio: unable to decode audio packet.");
				bytesRemaining = 0;
				Assert(0);
			}
			else
			{
				packet.data += bytesDecoded;
				packet.size -= bytesDecoded;
				
				bytesRemaining -= bytesDecoded;

				Assert(bytesDecoded >= 0);
				Assert(bytesRemaining >= 0);
				
				//swr_convert(m_swrContext, out, out_count, in, in_count);
				
				if (frameSize > 0)
				{
					if (m_codecContext->channels != 0)
					{
						size_t frameCount = frameSize / (m_codecContext->channels * sizeof(int16_t));

						segment.m_numSamples = frameCount * m_codecContext->channels;

						m_audioBuffer->AddSegment(segment);

						Assert(frameCount * (m_codecContext->channels * sizeof(int16_t)) == frameSize);

						Debug::Print("\t\tAudio: decoded frame. got %d frames.", int(frameCount));
					}
				}
				else
				{
					Assert(0);
				}
			}
		}
		
		av_frame_unref(frame);
		frame = nullptr;
		
		return true;
	}

	bool AudioContext::Depleted() const
	{
		return m_audioBuffer->Depleted() && (m_packetQueue->GetSize() == 0);
	}
};
