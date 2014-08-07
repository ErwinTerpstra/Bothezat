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