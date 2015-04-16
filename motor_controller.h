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

	struct PID
	{
		// PID coëfficients
		float kp;
		float ki;
		float kd;

		float integratedError;
		float lastError;

		PID();

		
	};

	Motor motors[Config::Constants::MC_MOTOR_AMOUNT];

	// PID values for all axes
	PID pid[3];

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