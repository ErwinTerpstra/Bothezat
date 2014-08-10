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

	Motor motors[Config::Constants::MC_MOTOR_AMOUNT];

protected:
	MotorController();

public:
	virtual void Setup();

	virtual void Loop(uint32_t dt);

private:
	void WriteMotor(const Motor& motor, uint16_t commmand);

	void EnablePWM();
	void EnablePin(uint8_t pin);
	void WritePwm(uint8_t pin, uint16_t dutyCycle);

};

}

#endif