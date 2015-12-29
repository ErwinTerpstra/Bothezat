#include "Arduino.h"
#include "bothezat.h"

#include "pwm_receiver.h"
#include "timer.h"

using namespace bothezat;

PwmReceiver::PwmReceiver() : Receiver(), timer(NULL)
{
	signalPins[0] = SignalPin(48, Receiver::CHANNEL1);
	signalPins[1] = SignalPin(49, Receiver::CHANNEL2);
	signalPins[2] = SignalPin(50, Receiver::CHANNEL3);
	signalPins[3] = SignalPin(51, Receiver::CHANNEL4);
	signalPins[4] = SignalPin(47, Receiver::CHANNEL5);
	signalPins[5] = SignalPin(46, Receiver::CHANNEL6);
}

void PwmReceiver::Setup()
{
	// Initialize interrupt channel
	pmc_enable_periph_clk(ID_PIOC);
	NVIC_DisableIRQ(PIOC_IRQn);
	NVIC_ClearPendingIRQ(PIOC_IRQn);
	NVIC_SetPriority(PIOC_IRQn, 0);
	NVIC_EnableIRQ(PIOC_IRQn);

	// Initialize all interrupts on all pins
	for (uint8_t pinIdx = 0; pinIdx < Config::Constants::RX_PWM_AMOUNT; ++pinIdx)
	{
		SignalPin& pin = signalPins[pinIdx];

		const PinDescription& desc = g_APinDescription[pin.pin];

		// Configure pin as input
		PIO_Configure(desc.pPort, PIO_INPUT, desc.ulPin, 0);

		// Enable interrupt on each edge change
		desc.pPort->PIO_AIMDR = pin.mask;
		desc.pPort->PIO_IER = pin.mask;
	}
	
	// Create a timer with a precision of at least 2 us
	timer = Timer::GetFreeTimer();
	uint16_t precision = timer->SetPrecision(2000);

	Debug::Print("Receiver timer set to %d ns precision\n", precision);

	timer->Start();
}

void PwmReceiver::Loop(uint32_t dt)
{
	uint16_t channels[Config::Constants::RX_MAX_CHANNELS];		// Pulse length for each channel

	// Initialize all channels to the mid level
	for (uint8_t channel = 0; channel < Config::Constants::RX_MAX_CHANNELS; ++channel)
		channels[channel] = config.RX_CHANNEL_CALIBRATION[channel].mid;

	// Copy pulse lengths from signal pins to channel array
	for (uint8_t pinIdx = 0; pinIdx < Config::Constants::RX_PWM_AMOUNT; ++pinIdx)
	{
		const SignalPin& signalPin = signalPins[pinIdx];
		channels[signalPin.channel] = signalPin.pulseLength;
	}
	
	// Save new pulse lengths
	UpdateChannels(channels);
}

void PwmReceiver::HandleISR(uint32_t mask)
{
	uint32_t time = timer->Micros();

	// Iterate through all signal pins the ones matching the interrupt mask
	for (uint8_t pinIdx = 0; pinIdx < Config::Constants::RX_PWM_AMOUNT; ++pinIdx)
	{
		if ((mask & signalPins[pinIdx].mask) != 0)
			signalPins[pinIdx].HandleChange(time);
	}
}

void PIOC_Handler(void) 
{
	PwmReceiver::Instance().HandleISR(PIOC->PIO_ISR);
}