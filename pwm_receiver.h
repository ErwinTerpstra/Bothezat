#ifndef _PWM_RECEIVER_H_
#define _PWM_RECEIVER_H_

#include "Arduino.h"
#include "receiver.h"
#include "timer.h"

namespace bothezat
{
	
/*
 *	Class which reads PWM output from radio receivers
 */
class PwmReceiver : public Receiver, public Module<PwmReceiver>
{
friend class Module<PwmReceiver>;

public:
	// Descriptor for a pin with a pwm pulse signal
	struct SignalPin
	{	
		uint16_t pin;
		uint32_t mask;

		Channel channel;
		uint32_t pulseLength;
		uint32_t pulseStart;

		SignalPin() { }

		SignalPin(uint16_t pin, Channel channel) : pin(pin), channel(channel), pulseLength(0), pulseStart(0)
		{
			mask = g_APinDescription[pin].ulPin;
		}

		SignalPin& operator=(const SignalPin& other)
		{
			if (&other == this)
				return *this;

			pin 			= other.pin;
			mask 			= other.mask;
			channel 		= other.channel;
			pulseLength 	= other.pulseLength;
			pulseStart 		= other.pulseStart;

			return *this;
		}
	};

private:
	Timer* timer;

	SignalPin signalPins[Config::Constants::RX_PWM_AMOUNT];

protected:
	PwmReceiver();

public:
	virtual void Setup();

	virtual void Loop(uint32_t dt);

	void HandleISR(uint32_t mask);

private:
	void ReadRaw();
};

}

#endif