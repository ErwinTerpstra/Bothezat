#include "Arduino.h"
#include "bothezat.h"

#include "pwm_receiver.h"
#include "timer.h"

using namespace bothezat;

PwmReceiver::PwmReceiver() : Receiver(), timer(NULL)
{
	// Mapping for Spektrum receivers
	mapping[Receiver::CHANNEL1] = Receiver::THROTTLE;
	mapping[Receiver::CHANNEL2] = Receiver::AILERON;
	mapping[Receiver::CHANNEL3] = Receiver::ELEVATOR;
	mapping[Receiver::CHANNEL4] = Receiver::RUDDER;
}

void PwmReceiver::Setup()
{
	Receiver::Setup();

	// Initialize all PWM pins to inputs
	for (uint8_t channel = 0; channel < Config::RX_PWM_AMOUNT; ++channel)
		pinMode(Config::Pins::RX_PWM + channel, INPUT);

	// Create a timer with a precision of at least 2 us
	timer = Timer::GetFreeTimer();
	timer->SetPrecision(2);
}

void PwmReceiver::Loop(uint32_t dt)
{
	Receiver::Loop(dt);

	uint16_t channels[Receiver::MAX_CHANNELS];				// Pulse length for each channel
	uint8_t channelsReading = Config::RX_PWM_AMOUNT;		// Amount of pin which are still being measured.

	// Initialize all channels to zero
	for (uint8_t channel = 0; channel < Receiver::MAX_CHANNELS; ++channel)
		channels[channel] = 0;

	// Disable interrupts to get accurate timing
	noInterrupts();

	uint8_t channel = 0;

	// Wait for all pins to be low
	do
	{
		for (channel = 0; channel < Config::RX_PWM_AMOUNT; ++channel)
		{
			// This pin is still high, break to signal the outer loop to start again
			// (Is it possible to wait here for the pin to go low? Or do we risk waiting to long then?)
			if (digitalRead(Config::Pins::RX_PWM + channel) == HIGH)
				break;
		}
	} while (channel < Config::RX_PWM_AMOUNT);

	// Read until a pulse has been measured for all pins
	do
	{	
		uint32_t start;

		// Find the first pin with a pulse
		for (channel = 0; channel < Config::RX_PWM_AMOUNT; ++channel)
		{
			if (digitalRead(Config::Pins::RX_PWM + channel) == HIGH)
			{
				// Start the timer here to minimize error
				timer->Start();
				break;
			}
		}

		// No pulse on any pins
		if (channel >= Config::RX_PWM_AMOUNT)
			continue;
		
		// Wait for the pulse to end
		while (digitalRead(Config::Pins::RX_PWM + channel) == HIGH);

		// Pulse is low again, save the timer value 
		channels[channel] = timer->Micros();
		--channelsReading;

		// Stop the timer
		timer->Stop();
	} while (channelsReading > 0);

	// Re-enable interrupts
	interrupts();

	// Convert amount of clock cycles to pulse lengths in microseconds
	//for (uint8_t channel = 0; channel < PinConfig::RX_PWM_AMOUNT; ++channel)
		//channels[channel] = (channels[channel] * 32) / (F_CPU / 1000000L);

	UpdateChannels(channels);
}