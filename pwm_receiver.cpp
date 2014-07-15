#include "Arduino.h"
#include "bothezat.h"

#include "pwm_receiver.h"

using namespace bothezat;

PwmReceiver::PwmReceiver() : Receiver()
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
	for (uint8_t channel = 0; channel < PinConfig::RX_PWM_AMOUNT; ++channel)
		pinMode(PinConfig::RX_PWM + channel, INPUT);

	// Enable the timer periphial we are going to use
  	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(ID_TC1);

	// Configure timer registers
	// TIMER_CLOCK1		MCK/2
	// TIMER_CLOCK2		MCK/8
	// TIMER_CLOCK3		MCK/32
	// TIMER_CLOCK4		MCK/128
}

void PwmReceiver::Loop(uint32_t dt)
{
	Receiver::Loop(dt);

	uint16_t channels[Receiver::MAX_CHANNELS];				// Pulse length for each channel
	uint8_t channelsReading = PinConfig::RX_PWM_AMOUNT;		// Amount of pin which are still being measured.

	// Initialize all channels to zero
	for (uint8_t channel = 0; channel < Receiver::MAX_CHANNELS; ++channel)
		channels[channel] = 0;

	// Disable interrupts to get accurate timing
	noInterrupts();

	TC_Configure(TC1, 0, TC_CMR_TCCLKS_TIMER_CLOCK3);
	uint8_t channel = 0;

	// Wait for all pins to be low
	do
	{
		for (channel = 0; channel < PinConfig::RX_PWM_AMOUNT; ++channel)
		{
			// This pin is still high, break to signal the outer loop to start again
			// (Is it possible to wait here for the pin to go low? Or do we risk waiting to long then?)
			if (digitalRead(PinConfig::RX_PWM + channel) == HIGH)
				break;
		}
	} while (channel < PinConfig::RX_PWM_AMOUNT);

	// Read until a pulse has been measured for all pins
	do
	{	
		// Find the first pin with a pulse
		for (channel = 0; channel < PinConfig::RX_PWM_AMOUNT; ++channel)
		{
			if (digitalRead(PinConfig::RX_PWM + channel) == HIGH)
			{
				// Start the timer here to minimize overhead
				TC_Start(TC1, 0);
				break;
			}
		}

		// No pulse on any pins
		if (channel >= PinConfig::RX_PWM_AMOUNT)
			continue;
		
		// Wait for the pulse to end
		while (digitalRead(PinConfig::RX_PWM + channel) == HIGH);

		// Pulse is low again, save the timer value 
		channels[channel] = TC_ReadCV(TC1, 0);
		--channelsReading;

		// Stop the timer
		TC_Stop(TC1, 0);
	} while (channelsReading > 0);

	// Re-enable interrupts
	interrupts();

	// Convert amount of clock cycles to pulse lengths in microseconds
	for (uint8_t channel = 0; channel < PinConfig::RX_PWM_AMOUNT; ++channel)
		channels[channel] = (channels[channel] * 32) / (F_CPU / 1000000L);

	/*
	channels[RUDDER]		= pulseIn(PinConfig::RX_PWM01, HIGH);
	channels[ELEVATOR]		= pulseIn(PinConfig::RX_PWM02, HIGH);
	channels[AILERON]		= pulseIn(PinConfig::RX_PWM03, HIGH);
	channels[THROTTLE]		= pulseIn(PinConfig::RX_PWM04, HIGH);
	*/

	UpdateChannels(channels);
}