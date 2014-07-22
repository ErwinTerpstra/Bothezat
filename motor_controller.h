#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include "Arduino.h"
#include "bothezat.h"

#include "module.h"

namespace bothezat
{

class MotorController : public Module<MotorController>
{
friend class Module<MotorController>;

public:
	struct Motor
	{
		bool enabled;
		uint8_t pin;

		Vector3 weights;
	};

	Motor motors[Config::MC_MOTOR_AMOUNT];

protected:
	MotorController();

public:
	virtual void Setup();

	virtual void Loop(uint32_t dt);
	
};

}

#endif