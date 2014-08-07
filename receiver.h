#ifndef _RECEIVER_H_
#define _RECEIVER_H_

#include "Arduino.h"

#include "module.h"

namespace bothezat
{
	
/*
 *	Base class for radio receivers
 */
 template <class Derived>
class Receiver : public Module<Derived>
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
		// Makes it easier to change the order
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
	bool connected;

protected:
	Receiver() : connected(false)
	{
		// Iterate through channels to initialize their values
		for (uint8_t channel = 0; channel < MAX_CHANNELS; ++channel)
		{
			channels[channel] = 0;
			mapping[channel] = (Channel) channel;	// Default channel order matches enum order
		}
	}
	
public:
	virtual void Setup()
	{

	}

	virtual void Loop(uint32_t dt)
	{
		
	}

	bool IsConnected() const { return connected; }

	void PrintChannels() const
	{
		Serial.println(F("Channels: "));
		Serial.print(channels[THROTTLE], DEC);
		Serial.print(F(", "));
		Serial.print(channels[ELEVATOR], DEC);
		Serial.print(F(", "));
		Serial.print(channels[AILERON], DEC);
		Serial.print(F(", "));
		Serial.print(channels[RUDDER], DEC);
		Serial.println(F(""));
		Serial.println(F(""));
	}

protected:

	void UpdateChannels(uint16_t* input)
	{
		// Apply channel mapping by copying channels from the input 
		for (uint8_t channel = 0; channel < MAX_CHANNELS; ++channel)
		{
			uint8_t target = mapping[channel];
			channels[target] = input[channel];
		}
	}

	void SetConnected(bool connected) { this->connected = connected; }

};

}

#endif