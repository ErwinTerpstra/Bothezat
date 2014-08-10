#include "Arduino.h"
#include "bothezat.h"
#include "timer.h"

using namespace bothezat;
 
const uint8_t Timer::CLOCK_DIVIDERS[] = 
{
	TC_CMR_TCCLKS_TIMER_CLOCK1,	2,
	TC_CMR_TCCLKS_TIMER_CLOCK2,	8,
	TC_CMR_TCCLKS_TIMER_CLOCK3,	32,
	TC_CMR_TCCLKS_TIMER_CLOCK4,	128
};

// Create all available timers
Timer Timer::timerPool[TIMER_AMOUNT] = 
{ 
	Timer(TC0, 0, TC0_IRQn),
	Timer(TC0, 1, TC1_IRQn),
	Timer(TC0, 2, TC2_IRQn),
	Timer(TC1, 0, TC3_IRQn),
	Timer(TC1, 1, TC4_IRQn),
	Timer(TC1, 2, TC5_IRQn),
	Timer(TC2, 0, TC6_IRQn),
	Timer(TC2, 1, TC7_IRQn),
	Timer(TC2, 2, TC8_IRQn)
};

Timer::Timer(Tc* timer, uint8_t channel, IRQn_Type irq) : 
	timer(timer), channel(channel), irq(irq), free(true)
{

}

Timer::Timer(const Timer& other)
{
	*this = other;
}

Timer& Timer::operator=(const Timer& other)
{
	if (&other == this)
		return *this;

	timer 		= other.timer;
	channel 	= other.channel;
	irq 		= other.irq;
	free 		= other.free;

	return *this;
}

void Timer::Start()
{
	NVIC_ClearPendingIRQ(irq);
	NVIC_EnableIRQ(irq);

	TC_Start(timer, channel);
}

void Timer::Stop()
{
	NVIC_DisableIRQ(irq);

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

	pmc_enable_periph_clk((uint32_t) irq);
	TC_Configure(timer, channel, flags);
}

uint32_t Timer::ReadValue() const
{
	return TC_ReadCV(timer, channel);
}

uint32_t Timer::SetPrecision(uint32_t desiredPrecision)
{
	uint32_t precision;
	uint8_t idx;

	for (idx = sizeof(CLOCK_DIVIDERS) - 1; idx > 1; idx -= 2)
	{
		precision = (1000 * CLOCK_DIVIDERS[idx]) / (VARIANT_MCK / 1000000);

		if (precision <= desiredPrecision)
			break;
	}

	divider = CLOCK_DIVIDERS[idx];
	Configure(CLOCK_DIVIDERS[idx - 1]);

	return precision;
}

uint32_t Timer::Micros() const
{
	// Use 64 bit to prevent overflow when multiplying
	uint64_t micros = ReadValue();
	micros = (micros * divider) / (VARIANT_MCK / 1000000);

	return (uint32_t) micros;
}

void TC0_Handler()
{
	TC_GetStatus(TC0, 0);
}

void TC1_Handler()
{
	TC_GetStatus(TC0, 1);
}

void TC2_Handler()
{
	TC_GetStatus(TC0, 2);
}

void TC3_Handler()
{
	TC_GetStatus(TC1, 0);
}

void TC4_Handler()
{
	TC_GetStatus(TC1, 1);
}

void TC5_Handler()
{
	TC_GetStatus(TC1, 2);
}

void TC6_Handler()
{
	TC_GetStatus(TC2, 0);
}

void TC7_Handler()
{
	TC_GetStatus(TC2, 1);
}

void TC8_Handler()
{
	TC_GetStatus(TC2, 2);
}