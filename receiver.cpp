#include "receiver.h"

using namespace bothezat;

Receiver* Receiver::currentReceiver = NULL;

Receiver::Receiver() : connected(false), config(Config::Instance())
{
	// Iterate through channels to initialize their values
	for (uint8_t channel = 0; channel < MAX_CHANNELS; ++channel)
	{
		channels[channel] = 0;
		mapping[channel] = (Channel) channel;	// Default channel order matches enum order
	}

	// TODO: read channel mapping from config

	SetSpektrumMapping();
}

void Receiver::SetSpektrumMapping()
{		
	// Mapping for Spektrum receivers
	mapping[Receiver::CHANNEL1] = Receiver::THROTTLE;
	mapping[Receiver::CHANNEL2] = Receiver::AILERON;
	mapping[Receiver::CHANNEL3] = Receiver::ELEVATOR;
	mapping[Receiver::CHANNEL4] = Receiver::RUDDER;
}

uint16_t Receiver::SerializeResource(Page::Resource::Type type, BinaryWriteStream& stream)
{
	switch (type)
	{
		case Page::Resource::RECEIVER_CHANNELS:
			for (uint8_t channel = 0; channel < MAX_CHANNELS; ++channel)
				stream.Write(channels[channel]);

			return sizeof(uint16_t) * MAX_CHANNELS;

		case Page::Resource::RECEIVER_NORMALIZED:
			for (uint8_t channel = 0; channel < MAX_CHANNELS; ++channel)
				stream.Write(NormalizedChannel(channels[channel]));

			return sizeof(float) * MAX_CHANNELS;

		case Page::Resource::RECEIVER_CONNECTED:
			stream.Write(connected);
			return 1;
	}

	return 0;
}

void Receiver::Debug() const
{
	Debug::Print("Channels:\n");
	Debug::Print("%d; %d; %d; %d\n", channels[THROTTLE], channels[ELEVATOR], channels[AILERON], channels[RUDDER]);
	Debug::Print("%.2f;%.2f;%.2f;%.2f\n", NormalizedChannel(THROTTLE), NormalizedChannel(ELEVATOR), NormalizedChannel(AILERON), NormalizedChannel(RUDDER));
}

// Returns a normalized, calibrated channel in the -1.0 ... 1.0f range
float Receiver::NormalizedChannel(Channel channel) const
{
	const Config::ChannelCalibration& calibration = config.RX_CHANNEL_CALIBRATION[channel];
	int16_t offset = channels[channel] - calibration.mid;

	if (abs(offset) < calibration.deadband)
		return 0.0f;

	if (offset > 0)
		return offset / (float) (calibration.max - calibration.mid);
	else
		return offset / (float) (calibration.mid - calibration.min);
}

void Receiver::UpdateChannels(uint16_t* input)
{
	// Apply channel mapping by copying channels from the input 
	for (uint8_t channel = 0; channel < MAX_CHANNELS; ++channel)
	{
		uint8_t target = mapping[channel];
		channels[target] = input[channel];
	}
}


bool Receiver::IsConnected() const { return connected; }

void Receiver::SetConnected(bool connected) { this->connected = connected; }