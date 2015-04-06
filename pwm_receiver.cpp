#include "Arduino.h"
#include "bothezat.h"

#include "pwm_receiver.h"
#include "timer.h"

using namespace bothezat;

PwmReceiver::PwmReceiver() : Receiver(), timer(NULL)
{
	signalPins[0] = SignalPin(48, 40, Receiver::CHANNEL1);
	signalPins[1] = SignalPin(49, 41, Receiver::CHANNEL2);
	signalPins[2] = SignalPin(50, 42, Receiver::CHANNEL3);
	signalPins[3] = SignalPin(51, 43, Receiver::CHANNEL4);
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

		pinMode(pin.debugPin, OUTPUT);

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
	//*
	ReadRaw();
	/*/

	uint16_t channels[Receiver::MAX_CHANNELS];				// Pulse length for each channel

	// Initialize all channels to zero
	for (uint8_t channel = 0; channel < Receiver::MAX_CHANNELS; ++channel)
		channels[channel] = 0;

	// Copy pulse lengths from signal pins to channel array
	for (uint8_t pinIdx = 0; pinIdx < Config::Constants::RX_PWM_AMOUNT; ++pinIdx)
	{
		const SignalPin& signalPin = signalPins[pinIdx];
		channels[signalPin.channel] = signalPin.pulseLength;
	}
	
	// Save new pulse lengths
	UpdateChannels(channels);
	//*/
}

void PwmReceiver::HandleISR(uint32_t mask)
{
	SignalPin* pin = NULL;

	// Iterate through all signal pins to find the one matching this interrupt mask
	for (uint8_t pinIdx = 0; pinIdx < Config::Constants::RX_PWM_AMOUNT; ++pinIdx)
	{
		if (mask == signalPins[pinIdx].mask)
		{
			pin = &signalPins[pinIdx];
			break;
		}
	}

	// No signal pin mapped to the interrupt pin
	if (pin == NULL)
		return;

	if (digitalRead(pin->pin))
	{
		// Start of pulse, save pulse start time
		pin->pulseStart = timer->Micros();
		digitalWrite(pin->debugPin, HIGH);
	}
	else
	{
		// End of pulse, save pulse length
		pin->pulseLength = timer->Micros() - pin->pulseStart;
		digitalWrite(pin->debugPin, LOW);
	}
}

void PwmReceiver::ReadRaw()
{
	uint16_t channels[Receiver::MAX_CHANNELS];						// Pulse length for each channel
	uint8_t channelsReading = Config::Constants::RX_PWM_AMOUNT;		// Amount of pin which are still being measured.

	// Initialize all channels to zero
	for (uint8_t channel = 0; channel < Receiver::MAX_CHANNELS; ++channel)
		channels[channel] = 0;

	// Disable interrupts to get accurate timing
	noInterrupts();

	uint8_t channel = 0;

	// Wait for all pins to be low
	do
	{
		for (channel = 0; channel < Config::Constants::RX_PWM_AMOUNT; ++channel)
		{
			// This pin is still high, break to signal the outer loop to start again
			// (Is it possible to wait here for the pin to go low? Or do we risk waiting to long then?)
			if (digitalRead(Config::Pins::RX_PWM + channel) == HIGH)
				break;
		}
	} while (channel < Config::Constants::RX_PWM_AMOUNT);

	// Read until a pulse has been measured for all pins
	do
	{	
		uint32_t start;

		// Find the first pin with a pulse
		for (channel = 0; channel < Config::Constants::RX_PWM_AMOUNT; ++channel)
		{
			if (digitalRead(Config::Pins::RX_PWM + channel) == HIGH)
			{
				// Start the timer here to minimize error
				timer->Start();
				break;
			}
		}

		// No pulse on any pins
		if (channel >= Config::Constants::RX_PWM_AMOUNT)
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

	UpdateChannels(channels);
}

void PIOC_Handler(void) 
{
	PwmReceiver::Instance().HandleISR(PIOC->PIO_ISR);
}