#include "Arduino.h"
#include "bothezat.h"

#include "motor_controller.h"
#include "receiver.h"
#include "motion_sensor.h"
#include "flight_system.h"

using namespace bothezat;

MotorController::MotorController() : receiver(NULL), motionSensor(NULL), flightSystem(NULL), armed(false)
{
	{
		// Front-right
		Motor& motor = motors[0];
		motor.enabled = true;
		motor.pin = 6;
		motor.weights = Vector3(-1.0f, 1.0f, -1.0f);
	}

	{
		// Front-left
		Motor& motor = motors[1];
		motor.enabled = true;
		motor.pin = 9;
		motor.weights = Vector3(-1.0f, -1.0f, 1.0f);
	}

	{
		// Back-left
		Motor& motor = motors[2];
		motor.enabled = true;
		motor.pin = 8;
		motor.weights = Vector3(1.0f, 1.0f, 1.0f);
	}

	{
		// Back-right
		Motor& motor = motors[3];
		motor.enabled = true;
		motor.pin = 7;
		motor.weights = Vector3(1.0f, -1.0f, -1.0f);
	}
}

void MotorController::Setup()
{
	receiver = &Receiver::CurrentReceiver();
	motionSensor = &MotionSensor::Instance();
	flightSystem = &FlightSystem::Instance();

	EnablePWM();

	// Enable all motors
	for (uint8_t motorIdx = 0; motorIdx < Config::Constants::MC_MOTOR_AMOUNT; ++motorIdx)
	{
		Motor& motor = motors[motorIdx];
		EnablePin(motor.pin);	
		WriteMotor(motor, config.MC_PWM_MIN_OUTPUT);
	}

	// Initialize all PID controllers
	for (uint8_t axis = 0; axis < 3; ++axis)
	{
		PidController& pid = pidControllers[axis];
		pid.Configure(config.MC_PID_CONFIGURATION[axis]);
	}

	pidControllers[1].enabled = false;
}

void MotorController::Loop(uint32_t dt)
{	
	if (!IsArmed())
		return;
	
	const Quaternion& orientation = motionSensor->CurrentOrientation();
	const Rotation& desiredRotation = flightSystem->CurrentMode().DesiredRotation();
	Rotation rotation;

	// Convert the quaternion orientation to yaw pitch roll rotation
	orientation.ToEulerAngles(rotation);

	float deltaSeconds = dt * 1e-6f;

	// Update the PidController controllers for each axis
	for (uint8_t axis = 0; axis < 3; ++axis)
	{
		PidController& pid = pidControllers[axis];

		pid.target = desiredRotation[axis] / 180.0f;
		pid.Update(rotation[axis] / 180.0f, deltaSeconds);
	}

	UpdateMotorsRelative();
}

void MotorController::UpdateMotorsRelative()
{	
	// Base point and output multiplier for each motor is determined by throttle
	float throttle = receiver->NormalizedChannel(Receiver::THROTTLE);
	throttle = (throttle + 1.0f) * 0.5f;

	// Calculate outputs for each motor
	for (uint8_t motorIdx = 0; motorIdx < Config::Constants::MC_MOTOR_AMOUNT; ++motorIdx)
	{
		Motor& motor = motors[motorIdx];

		motor.lastOutput = throttle;

		// Add the output for each axis according to the motor weight for that axis
		for (uint8_t axis = 0; axis < 3; ++axis)
			motor.lastOutput += pidControllers[axis].output * motor.weights[axis] * throttle;

		// Clamp within 0 ... 1 range 
		motor.lastOutput = Util::Clamp(motor.lastOutput, 0.0f, 1.0f);

		// Convert the output to a PWM command
		uint16_t command = config.MC_PWM_MIN_COMMAND + (config.MC_PWM_MAX_COMMAND - config.MC_PWM_MIN_COMMAND) * motor.lastOutput;

		// Write the output for this motor
		WriteMotor(motor, command);
	}
}

void MotorController::UpdateMotorsNormalized()
{	

	// Output multiplier for each motor is determined by throttle amount
	float throttle = receiver->NormalizedChannel(Receiver::THROTTLE);
	throttle = (throttle + 1.0f) * 0.5f;

	float totalOutput = 0.0f;
	float maxOutput = 0.0f;

	// Calculate outputs for each motor
	for (uint8_t motorIdx = 0; motorIdx < Config::Constants::MC_MOTOR_AMOUNT; ++motorIdx)
	{
		Motor& motor = motors[motorIdx];

		float output = 0.0f;

		// Add the output for each axis according to the motor weight for that axis
		for (uint8_t axis = 0; axis < 3; ++axis)
			output += pidControllers[axis].output * motor.weights[axis];

		motor.lastOutput = output;

		totalOutput += output;
		maxOutput = max(maxOutput, output);
	}

	// Normalize and apply outputs for each motor
	for (uint8_t motorIdx = 0; motorIdx < Config::Constants::MC_MOTOR_AMOUNT; ++motorIdx)
	{
		Motor& motor = motors[motorIdx];

		// Normalize output against the maximum motor output. 
		// This means the motor with the highest value will receive output according to the throttle value
		// All other motor outputs are scaled accordingly
		motor.lastOutput = (motor.lastOutput / maxOutput) * throttle;

		// Clamp within 0 ... 1 range (Higher than 1 should not occur, TODO: test this)
		// NOTE: negative values here could be used as input for active brakes?
		motor.lastOutput = Util::Clamp(motor.lastOutput, 0.0f, 1.0f);

		// Convert the output to a PWM command
		uint16_t command = config.MC_PWM_MIN_COMMAND + (config.MC_PWM_MAX_COMMAND - config.MC_PWM_MIN_COMMAND) * motor.lastOutput;

		// Write the output for this motor
		WriteMotor(motor, command);
	}
}

void MotorController::Debug()
{
	Debug::Print("PidController:\n");

	for (uint8_t axis = 0; axis < 3; ++axis)
	{
		const PidController& pid = pidControllers[axis];
		Debug::Print("%u: Input: %.3f; Target: %.3f; Output: %.3f; Last error: %.3f; Integrated error: %.3f;\n", 
					  axis, pid.lastInput, pid.target, pid.output, pid.lastError, pid.integratedError);
	}

	Debug::Print("\n");

	Debug::Print("Motor commands:\n");

	for (uint8_t motorIdx = 0; motorIdx < Config::Constants::MC_MOTOR_AMOUNT; ++motorIdx)
	{
		const Motor& motor = motors[motorIdx];
		Debug::Print("%u: %u\n", motorIdx, motor.lastCommand);
	}
}

void MotorController::SetArmState(bool state)
{
	if (state == armed)
		return;

	armed = state;

	if (!armed)
	{
		DisableMotors();
		ResetControllers();
		Debug::Print("Motors disarmed!\n");
	}
	else
		Debug::Print("Motors armed!\n");
}

void MotorController::DisableMotors()
{
	// Write minimum commands to all motors
	for (uint8_t motorIdx = 0; motorIdx < Config::Constants::MC_MOTOR_AMOUNT; ++motorIdx)
	{
		Motor& motor = motors[motorIdx];
		WriteMotor(motor, config.MC_PWM_MIN_OUTPUT);
	}
}

void MotorController::ResetControllers()
{
	// Reset all PID controllers
	for (uint8_t axis = 0; axis < 3; ++axis)
	{
		PidController& pid = pidControllers[axis];
		pid.Reset();
	}
}

void MotorController::WriteMotor(Motor& motor, uint16_t command)
{
	if (!motor.enabled)
		return;

	motor.lastCommand = Util::Clamp(command, config.MC_PWM_MIN_OUTPUT, config.MC_PWM_MAX_COMMAND);
	WritePwm(motor.pin, motor.lastCommand);
}

void MotorController::EnablePWM()
{
	Debug::Print("Enabling PWM periphial...\n");

    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    PWMC_ConfigureClocks(config.MC_PWM_FREQUENCY * config.MC_PWM_PERIOD, 0, VARIANT_MCK);
}

void MotorController::EnablePin(uint8_t pin)
{
	const PinDescription& desc = g_APinDescription[pin];
    uint32_t channel = desc.ulPWMChannel;

    // Make sure this is a PWM pin
    assert((desc.ulPinAttribute & PIN_ATTR_PWM) == PIN_ATTR_PWM);
	
	// Disable the channel so we can directly write to the registers
	PWMC_DisableChannel(PWM_INTERFACE, channel);
	while ((PWM_INTERFACE->PWM_SR & (1 << channel)) != 0);	// Wait for the channel to be disabled

	PIO_Configure(desc.pPort, desc.ulPinType, desc.ulPin, desc.ulPinConfiguration);

	// Configure channel for our frequencies
	PWMC_ConfigureChannel(PWM_INTERFACE, channel, PWM_CMR_CPRE_CLKA, 0, 0);
	PWMC_SetPeriod(PWM_INTERFACE, channel, config.MC_PWM_PERIOD);
	PWMC_SetDutyCycle(PWM_INTERFACE, channel, 0);

	// Enable the channel again
	PWMC_EnableChannel(PWM_INTERFACE, channel);

	Debug::Print("Motor on pin %u enabled\n", pin);
}

void MotorController::WritePwm(uint8_t pin, uint16_t dutyCycle)
{
	const PinDescription& desc = g_APinDescription[pin];
	PWMC_SetDutyCycle(PWM_INTERFACE, desc.ulPWMChannel, dutyCycle);
}

MotorController::PidController::PidController() : enabled(true), kp(1.0f), ki(1.0f), kd(1.0f), 
	integratedError(0.0f), lastError(0.0f), target(0.0f), output(0.0f), lastInput(0.0f)
{

}

void MotorController::PidController::Configure(Config::PidConfiguration configuration)
{
	kp = configuration.kp;
	ki = configuration.ki;
	kd = configuration.kd;
}

void MotorController::PidController::Update(float input, float dt)
{
	if (!enabled)
		return;

	float error = target - input;
	integratedError += error * dt;

	//float deltaError = (error - lastError) / dt;
	float deltaInput = (input - lastInput) / dt;
	output = kp * error + ki * integratedError - kd * deltaInput;

	lastError = error;
	lastInput = input;
}

void MotorController::PidController::Reset()
{
	integratedError = 0.0f;
	lastError = 0.0f;

	target = 0.0f;
	output = 0.0f;
	lastInput = 0.0f;
}