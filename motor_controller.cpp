#include "Arduino.h"
#include "bothezat.h"

#include "motor_controller.h"
#include "pwm_receiver.h"

using namespace bothezat;

MotorController::MotorController()
{
	{
		// Front-right
		Motor& motor = motors[0];
		motor.enabled = true;
		motor.pin = 6;
		motor.weights = Vector3(-1.0f, 1.0f, 1.0f);
	}

	/*
	{
		// Front-left
		Motor& motor = motors[1];
		motor.enabled = true;
		motor.pin = 7;
		motor.weights = Vector3(-1.0f, -1.0f, -1.0f);
	}

	{
		// Back-left
		Motor& motor = motors[2];
		motor.enabled = true;
		motor.pin = 8;
		motor.weights = Vector3(1.0f, 1.0f, -1.0f);
	}

	{
		// Back-right
		Motor& motor = motors[3];
		motor.enabled = true;
		motor.pin = 9;
		motor.weights = Vector3(1.0f, -1.0f, 1.0f);
	}
	*/
}

void MotorController::Setup()
{
	EnablePWM();

	for (uint8_t motorIdx = 0; motorIdx < Config::Constants::MC_MOTOR_AMOUNT; ++motorIdx)
	{
		const Motor& motor = motors[motorIdx];
		EnablePin(motor.pin);	
	}

	//delay(10);
}

void MotorController::Loop(uint32_t dt)
{
	PwmReceiver& receiver = PwmReceiver::Instance();
	WriteMotor(motors[0], receiver.channels[PwmReceiver::THROTTLE]);
}

void MotorController::WriteMotor(const Motor& motor, uint16_t command)
{
	WritePwm(motor.pin, min(command, config.MC_PWM_MAX_COMMAND));
}

void MotorController::EnablePWM()
{
	Debug::Print("Enabling PWM periphial...\n");

    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    PWMC_ConfigureClocks(config.MC_PWM_FREQUENCY * config.MC_PWM_MAX_COMMAND, 0, VARIANT_MCK);
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

	// Configre channel for our frequencies
	PWMC_ConfigureChannel(PWM_INTERFACE, channel, PWM_CMR_CPRE_CLKA, 0, 0);
	PWMC_SetPeriod(PWM_INTERFACE, channel, config.MC_PWM_MAX_COMMAND);
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