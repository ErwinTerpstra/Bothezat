#ifndef _RECEIVER_H_
#define _RECEIVER_H_

#include "Arduino.h"

#include "module.h"

namespace bothezat
{
	
/*
 *	Base class for radio receivers
 */
class Receiver 
{
public:
	enum Channel
	{
		// All available RC channels, if more should be read the need to be added below
		AILERON,
		ELEVATOR,
		THROTTLE,
		RUDDER,
		AUX1, AUX2,
		AUX3, AUX4,

		MAX_CHANNELS,

		// Generalization of default order
		// These are used for easy channel mapping
		CHANNEL1 = AILERON,
		CHANNEL2 = ELEVATOR,
		CHANNEL3 = THROTTLE,
		CHANNEL4 = RUDDER,
		CHANNEL5 = AUX1,
		CHANNEL6 = AUX2,
		CHANNEL7 = AUX3,
		CHANNEL8 = AUX4
	};

	// Values of all channels as last set in UpdateChannels()
	uint16_t channels[MAX_CHANNELS];

	// Mapping of input signals and the way signals are saved in UpdateChannels
	Channel mapping[MAX_CHANNELS];

private:
	static Receiver* currentReceiver;

	const Config& config;

	bool connected;

protected:
	Receiver();

	void SetSpektrumMapping();
	
public:

	virtual void Setup() = 0;
	virtual void Loop(uint32_t dt) = 0;

	virtual void Debug() const;

	bool IsConnected() const;

	// Returns a normalized, calibrated channel in the -1.0 ... 1.0f range
	float NormalizedChannel(Channel channel) const;

	static void SetReceiver(Receiver& receiver) { currentReceiver = &receiver; }
	static Receiver& CurrentReceiver() { return *currentReceiver; }

protected:

	void UpdateChannels(uint16_t* input);
	void SetConnected(bool connected);

};

}

#endif