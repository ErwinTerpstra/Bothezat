#include "timer.h"

using namespace bothezat;
 
const uint8_t Timer::CLOCK_DIVIDERS[]= 
{
	TC_CMR_TCCLKS_TIMER_CLOCK1,	2,
	TC_CMR_TCCLKS_TIMER_CLOCK2,	8,
	TC_CMR_TCCLKS_TIMER_CLOCK3,	32,
	TC_CMR_TCCLKS_TIMER_CLOCK4,	128
};

Timer::Timer(Tc* timer, uint8_t channel) : timer(timer), channel(channel)
{
}

void Timer::Start()
{
	TC_Start(timer, channel);
}

void Timer::Stop()
{
	TC_Stop(timer, channel);
}

void Timer::Configure(uint32_t flags)
{
	// Configure timer registers
	// TC_CMR_TCCLKS_TIMER_CLOCK3
	// TIMER_CLOCK1		MCK/2
	// TIMER_CLOCK2		MCK/8
	// TIMER_CLOCK3		MCK/32
	// TIMER_CLOCK4		MCK/128

	TC_Configure(timer, channel, flags);
}

uint64_t Timer::ReadValue() const
{
	return TC_ReadCV(timer, channel);
}

uint16_t Timer::SetPrecision(uint16_t desiredPrecision)
{
	uint16_t precision;
	uint8_t idx;

	for (idx = sizeof(CLOCK_DIVIDERS) - 1; idx > 1; idx -= 2)
	{
		divider = CLOCK_DIVIDERS[idx];
		precision = (1000000L * divider) / F_CPU;

		if (precision <= desiredPrecision)
			break;
	}

	Configure(CLOCK_DIVIDERS[idx - 1]);

	return precision;
}

uint64_t Timer::Micros() const
{
	return (ReadValue() * divider) / (F_CPU / 1000000L);
}