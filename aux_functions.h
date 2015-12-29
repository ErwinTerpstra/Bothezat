#ifndef _AUX_FUNCTIONS_H_
#define _AUX_FUNCTIONS_H_

#include "Arduino.h"
#include "bothezat.h"
#include "module.h"

#include "receiver.h"

namespace bothezat
{

class FlightSystem;
class MotorController;

class AuxFunctions : public Module<AuxFunctions>
{
friend class Module<AuxFunctions>;

public:

	struct ControlFunction
	{
		enum Type
		{
			UNKNOWN,

			ENABLE_ANGLE_MODE,
			ARM_MOTORS,
		};

		Type type;

		Receiver::Channel channel;

		float min, max;

		bool active;

		ControlFunction() : active(false), type(UNKNOWN)
		{
			min = 0.0f;
			max = 1.0f;
		}
	};

	ControlFunction functions[Config::Constants::AC_MAX_CONTROL_FUNCTIONS];

	uint8_t registeredFunctions;

	Receiver* receiver;

	FlightSystem* flightSystem;

	MotorController* motorController;

protected:
	AuxFunctions();

public:
	virtual void Setup();

	virtual void Loop(uint32_t dt);

	void AddControlFunction(ControlFunction& function);

private:

	void ActivateFunction(ControlFunction& function);
	void DeactivateFunction(ControlFunction& function);

};

}


#endif