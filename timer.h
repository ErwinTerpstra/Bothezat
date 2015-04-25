#ifndef _TIMER_H_
#define _TIMER_H_

#include "Arduino.h"
#include "debug.h"

namespace bothezat
{
	
class Timer
{
public:

	static const uint8_t CLOCK_DIVIDERS[];

	static const uint8_t TIMER_AMOUNT = 9;

private:
	static Timer timerPool[TIMER_AMOUNT];

	Tc* timer;
	uint8_t channel;
	IRQn_Type irq;

	uint16_t divider;

	bool free;

public:
	void Start();
	void Stop();
	uint32_t ReadValue() const;
	void Configure(uint32_t flags);

	uint32_t SetPrecision(uint32_t desiredPrecision);
	uint32_t Micros() const;

	Timer& operator=(const Timer& other);
private:
	Timer(const Timer& other);
	Timer(Tc* timer, uint8_t channel, IRQn_Type irq);

public:
	static void EnableTimers()
	{
		Debug::Print("Enabling timer periphial...\n");
		
		// Enable the timer periphial we are going to use
	  	pmc_set_writeprotect(false);
		pmc_enable_periph_clk(ID_TC0);
		pmc_enable_periph_clk(ID_TC1);
		pmc_enable_periph_clk(ID_TC2);
	}

	static Timer* GetFreeTimer()
	{
		Timer* timer = NULL;

		for (uint8_t timerIdx = 0; timerIdx < TIMER_AMOUNT; ++timerIdx)
		{
			timer = &timerPool[timerIdx];

			if (timer->free)
			{
				Debug::Print("Using timer %u\n", timerIdx);
				timer->free = false;
				return timer;
			}
		}

		assert(false && "No more free timers available!");

		return NULL;
	}

	static void ReleaseTimer(Timer* timer)
	{
		timer->free = true;
	}


};

}

#endif;