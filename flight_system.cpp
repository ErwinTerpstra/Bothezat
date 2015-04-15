#include "Arduino.h"
#include "flight_system.h"

using namespace bothezat;

FlightSystem::FlightSystem()
{
	Receiver& receiver = Receiver::CurrentReceiver();

	flightModes[MANUAL]		= new ManualMode(receiver);
	flightModes[ATTITUDE]	= new AttitudeMode(receiver);

	currentMode = flightModes[ATTITUDE];
	currentMode->OnEnter();
}

void FlightSystem::Setup()
{
	for (uint8_t modeIdx = 0; modeIdx < LAST_FLIGHT_MODE; ++modeIdx)
		flightModes[modeIdx]->Setup();
}

void FlightSystem::Debug() const
{
	float yaw, pitch, roll;
	currentMode->DesiredOrientation().ToEulerAngles(yaw, pitch, roll);
	Debug::Print("Desired orientation:\n");
	Debug::Print("Yaw = %.4f; Pitch = %.4f; Roll = %.4f\n", yaw, pitch, roll);
}

void FlightSystem::Loop(uint32_t dt)
{
	currentMode->Loop(dt);
}
	
void FlightSystem::SwitchMode(FlightMode* newMode)
{
	currentMode->OnExit();
	currentMode = newMode;
	currentMode->OnEnter();
}