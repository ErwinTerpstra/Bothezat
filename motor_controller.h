#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include "Arduino.h"
#include "bothezat.h"

#include "module.h"

namespace bothezat
{

class MotionSensor;
class Receiver;
class FlightSystem;

class MotorController : public Module<MotorController>
{
friend class Module<MotorController>;

public:
	struct Motor
	{
		bool enabled;
		uint8_t pin;

		float lastOutput;
		uint16_t lastCommand;

		Vector3 weights;

		Motor() : enabled(true), lastOutput(0.0f), lastCommand(0), weights(0.0f, 0.0f, 0.0f)
		{

		}
	};

	struct PidController
	{
		bool enabled;

		// PID coëfficients
		float kp;
		float ki;
		float kd;

		float integratedError;
		float lastError;

		float target;
		float output;
		float lastInput;

		PidController();

		void Configure(Config::PidConfiguration configuration);

		void Update(float input, float dt);

		void Reset();
		
	};

private:
	Motor motors[Config::Constants::MC_MOTOR_AMOUNT];

	// PID values for all axes
	PidController pidControllers[3];

	MotionSensor* motionSensor;
	Receiver* receiver;
	FlightSystem* flightSystem;	

	bool armed;

protected:
	MotorController();

public:
	virtual void Setup();

	virtual void Loop(uint32_t dt);

	virtual void Debug();

	void SetArmState(bool state);

	void DisableMotors();
	void ResetControllers();

	bool IsArmed() const { return armed; }

private:
	void UpdateMotorsRelative();
	void UpdateMotorsNormalized();

	void WriteMotor(Motor& motor, uint16_t commmand);

	void EnablePWM();
	void EnablePin(uint8_t pin);
	void WritePwm(uint8_t pin, uint16_t dutyCycle);

};

}

#endif