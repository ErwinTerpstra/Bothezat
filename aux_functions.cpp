#include "aux_functions.h"

#include "flight_modes.h"

#include "receiver.h"
#include "flight_system.h"
#include "motor_controller.h"


using namespace bothezat;

AuxFunctions::AuxFunctions() : registeredFunctions(0), receiver(NULL), flightSystem(NULL), motorController(NULL)
{
	
}

void AuxFunctions::Setup()
{
	receiver = &Receiver::CurrentReceiver();
	flightSystem = &FlightSystem::Instance();
	motorController = &MotorController::Instance();
}

void AuxFunctions::Loop(uint32_t dt)
{
	for (uint8_t functionIdx = 0; functionIdx < registeredFunctions; ++functionIdx)
	{
		ControlFunction& function = functions[functionIdx];
		float value = receiver->NormalizedChannel(function.channel);
		bool inRange = value >= function.min && value <= function.max;

		if (inRange && !function.active)
			ActivateFunction(function);
		else if (!inRange && function.active)
			DeactivateFunction(function);
	}
}

void AuxFunctions::AddControlFunction(ControlFunction& function)
{
	if (registeredFunctions >= Config::Constants::AC_MAX_CONTROL_FUNCTIONS)
		return;

	functions[registeredFunctions++] = function;
}

void AuxFunctions::ActivateFunction(ControlFunction& function)
{
	switch (function.type)
	{
		case ControlFunction::ENABLE_ANGLE_MODE:
			flightSystem->SwitchMode(FlightMode::ANGLE);
			break;

		case ControlFunction::ARM_MOTORS:
			motorController->SetArmState(true);
			break;
	}

	function.active = true;
}

void AuxFunctions::DeactivateFunction(ControlFunction& function)
{
	switch (function.type)
	{
		case ControlFunction::ENABLE_ANGLE_MODE:
			flightSystem->SwitchMode(FlightSystem::DEFAULT_MODE);
			break;

		case ControlFunction::ARM_MOTORS:
			motorController->SetArmState(false);
			break;
	}

	function.active = false;
}