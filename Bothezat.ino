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
#include "timer.h"

using namespace bothezat;

MotionSensor& motionSensor = MotionSensor::Instance();
PwmReceiver& receiver = PwmReceiver::Instance();
LedController& ledController = LedController::Instance();
MotorController& motorController = MotorController::Instance();

Timer timer(TC0, 0);
uint32_t dt, debugTime;
uint64_t loopStart, lastLoopStart, loopEnd;
const uint16_t loopTime = 0;

void setup()
{
	Serial.begin(115200);
	Serial.println("======== Bothezat ========");
	Serial.println("Initializing...");

	Timer::EnableTimers();

	I2C::Setup();

	motionSensor.Setup();
	receiver.Setup();
	ledController.Setup();
	motorController.Setup();

	uint16_t precision = timer.SetPrecision(2);
	Debug::Print("Timer set to %d us precision\n", precision);

	timer.Start();

	dt = 10;
	debugTime = 0;
	loopStart = lastLoopStart = timer.Micros();

	Serial.println("Initialization complete!");
}

void loop()
{
	loopStart = timer.Micros();

	// Overflow, don't update deltatime
	if (lastLoopStart < loopStart)
	{
		dt = loopStart - lastLoopStart;
		lastLoopStart = loopStart;
	}
	else
	{
		Serial.println(F("Overflow!"));
	}

	motionSensor.Loop(dt);
	//receiver.Loop(dt);
	//ledController.Loop(dt);
	motorController.Loop(dt);

	debugTime += dt;

	if (debugTime >= 1000000L)
	{
		motionSensor.PrintOrientation();
		receiver.PrintChannels();
		Serial.println(dt);
		debugTime = 0;
	}

	loopEnd = timer.Micros();

	delay(constrain(loopTime - ((loopEnd - loopStart) / 1000), 0, loopTime));
}
