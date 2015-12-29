#include "Arduino.h"
#include "flight_system.h"

using namespace bothezat;

FlightSystem::FlightSystem()
{
	Receiver& receiver = Receiver::CurrentReceiver();

	flightModes[FlightMode::MANUAL]		= new ManualMode(receiver);
	flightModes[FlightMode::ANGLE]		= new AngleMode(receiver);

	currentMode = DEFAULT_MODE;
	CurrentMode().OnEnter();
}

void FlightSystem::Setup()
{
	for (uint8_t modeIdx = 0; modeIdx < FlightMode::LAST_FLIGHT_MODE; ++modeIdx)
		flightModes[modeIdx]->Setup();
}

void FlightSystem::Debug() const
{
	float yaw, pitch, roll;
	CurrentMode().DesiredOrientation().ToEulerAngles(yaw, pitch, roll);
	
	Debug::Print("Desired orientation:\n");
	Debug::Print("Yaw = %.4f; Pitch = %.4f; Roll = %.4f\n", yaw, pitch, roll);
}

void FlightSystem::Loop(uint32_t dt)
{
	CurrentMode().Loop(dt);
}

void FlightSystem::SwitchMode(FlightMode::ID id)
{
	if (id == currentMode)
		return;

	CurrentMode().OnExit();

	currentMode = id;

	CurrentMode().OnEnter();

	Debug::Print("Switched flight mode to: %s\n", CurrentMode().Name());
}