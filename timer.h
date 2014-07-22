#ifndef _TIMER_H_
#define _TIMER_H_

#include "Arduino.h"

namespace bothezat
{
	
class Timer
{
public:

	static const uint8_t CLOCK_DIVIDERS[];

private:
	Tc* timer;
	const uint8_t channel;

	uint16_t divider;

public:
	Timer(Tc* timer, uint8_t channel);

	void Start();
	void Stop();
	uint64_t ReadValue() const;
	void Configure(uint32_t flags);

	uint16_t SetPrecision(uint16_t desiredPrecision);
	uint64_t Micros() const;

	static void EnableTimers()
	{
		// Enable the timer periphial we are going to use
	  	pmc_set_writeprotect(false);
		pmc_enable_periph_clk(ID_TC0);
		pmc_enable_periph_clk(ID_TC1);
		pmc_enable_periph_clk(ID_TC2);
	}


};

}

#endif;