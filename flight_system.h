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
	static const FlightMode::ID DEFAULT_MODE = FlightMode::MANUAL;

private:
	FlightMode* flightModes[FlightMode::LAST_FLIGHT_MODE];

	FlightMode::ID currentMode;

protected:
	FlightSystem();

public:

	virtual void Setup();
	virtual void Loop(uint32_t dt);
	virtual void Debug() const;

	void SwitchMode(FlightMode::ID id);

	FlightMode& CurrentMode() { return *flightModes[currentMode]; }
	const FlightMode& CurrentMode() const { return *flightModes[currentMode]; }

	FlightMode::ID CurrentModeID() { return currentMode;}

};

}

#endif