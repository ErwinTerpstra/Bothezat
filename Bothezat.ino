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

void setup()
{
	Serial.begin(115200);

	I2C::Setup();

	motionSensor.Setup();
	receiver.Setup();
	ledController.Setup();
}

void loop()
{
	uint32_t loopStart = micros();

	motionSensor.Loop();
	receiver.Loop();
	ledController.Loop();
	motorController.Loop();

	uint32_t loopEnd = micros();

	Serial.print(F("Loop time: "));
	Serial.print((loopEnd - loopStart) / 1000, 1);
	Serial.println(F("ms"));

	delay(1000);
}