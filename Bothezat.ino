// We have to include all the libraries the modules use here so that their paths get added to the compilers include path
// Arduino tool chain sucks..
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include "bothezat.h"

#include "i2c.h"
#include "motion_sensor.h"
#include "pwm_receiver.h"
#include "motor_controller.h"
#include "led_controller.h"

using namespace bothezat;

MotionSensor& motionSensor = MotionSensor::Instance();
PwmReceiver& receiver = PwmReceiver::Instance();
LedController& ledController = LedController::Instance();
MotorController& motorController = MotorController::Instance();

uint32_t dt, timer;
uint64_t loopStart, lastLoopStart, loopEnd;

void setup()
{
	Serial.begin(115200);
	Serial.println("======== Bothezat ========");
	Serial.print("Initializing...");

	I2C::Setup();

	motionSensor.Setup();
	receiver.Setup();
	ledController.Setup();

	dt = 0;
	timer = 0;
	loopStart = lastLoopStart = micros();

	Serial.println(" Complete!");
}

void loop()
{
	loopStart = micros();

	// Overflow, don't update deltatime
	if (lastLoopStart < loopStart)
	{
		dt = loopStart - lastLoopStart;
		lastLoopStart = loopStart;
	}

	motionSensor.Loop(dt);
	receiver.Loop(dt);
	ledController.Loop(dt);
	motorController.Loop(dt);

	timer += dt;

	if (timer >= 1000000L)
	{
		motionSensor.PrintOrientation();
		Serial.println(dt, DEC);

		timer = 0;
	}

	loopEnd = micros();

	delay(constrain(10 - ((loopEnd - loopStart) / 1000), 1, 10));
}