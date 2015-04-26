#ifndef _FLIGHT_SYSTEM_H_
#define _FLIGHT_SYSTEM_H_

#include "Arduino.h"
#include "module.h"

#include "flight_modes.h"

namespace bothezat
{
	
class FlightSystem : public Module<FlightSystem>
{
friend class Module<FlightSystem>;

public:

private:
	FlightMode* flightModes[LAST_FLIGHT_MODE];

	FlightMode* currentMode;

protected:
	FlightSystem();

public:

	virtual void Setup();
	virtual void Loop(uint32_t dt);
	virtual void Debug() const;

	void SwitchMode(FlightMode* newMode);
	const FlightMode& CurrentMode() { return *currentMode; }

};

}

#endif