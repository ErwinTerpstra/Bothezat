// We have to include all the libraries the modules use here so that their paths get added to the compilers include path
// Arduino tool chain sucks..
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include "bothezat.h"

#include "i2c.h"
#include "motion_sensor.h"
#include "receiver.h"
#include "pwm_receiver.h"
#include "flight_system.h"
#include "motor_controller.h"
#include "led_controller.h"
#include "serial_interface.h"
#include "timer.h"

using namespace bothezat;


class Core
{

private:


	Timer* timer;

	uint32_t dt, debugTime;
	uint32_t loopStart, lastLoopStart, loopEnd;

	Config& config;

	// Modules
	Receiver* receiver;
	MotionSensor* motionSensor;
	LedController* ledController;
	FlightSystem* flightSystem;
	MotorController* motorController;
	SerialInterface* serialInterface;

public:
	Core() : timer(NULL), config(Config::Instance())
	{

	}

	void Setup()
	{		
		config.ReadEEPROM();

		Util::Init();
		I2C::Setup();

		// Initialize serial interface first so that we can begin sending messages
		serialInterface = &SerialInterface::Instance();
		serialInterface->Setup();

		Debug::Print("======== Bothezat ========\n");
		Debug::Print("Initializing...\n");

		// Initialize timer
		Timer::EnableTimers();

		// Construct other modules
		Receiver::SetReceiver(PwmReceiver::Instance());

		receiver = &Receiver::CurrentReceiver();
		motionSensor = &MotionSensor::Instance();
		flightSystem = &FlightSystem::Instance();
		//ledController = &LedController::Instance();
		motorController = &MotorController::Instance();

		// Initialize modules
		motionSensor->Setup();
		receiver->Setup();
		flightSystem->Setup();
		//ledController->Setup();
		motorController->Setup();

		timer = Timer::GetFreeTimer();

		uint16_t precision = timer->SetPrecision(1000);
		Debug::Print("Main timer set to %d ns precision\n", precision);

		timer->Start();

		dt = 10;
		debugTime = 0;
		loopStart = lastLoopStart = timer->Micros();

		Debug::Print("Initialization complete!\n");
	}

	void Loop()
	{
		loopStart = timer->Micros();

		// Don't update deltatime on an overflow
		if (lastLoopStart <= loopStart)
			dt = loopStart - lastLoopStart;
		
		lastLoopStart = loopStart;

		motionSensor->Loop(dt);
		receiver->Loop(dt);
		//ledController->Loop(dt);
		flightSystem->Loop(dt);
		motorController->Loop(dt);
		//serialInterface->Loop(dt);

		debugTime += dt;

		if (debugTime >= 500000L)
		{
			motionSensor->Debug();
			receiver->Debug();
			flightSystem->Debug();
			motorController->Debug();

			Debug::Print("Last loop time: %dus\n", dt);
			Debug::Print("Uptime: %ds\n", timer->Micros() / 1000000);
			
			debugTime = 0;
		}

		loopEnd = timer->Micros();

		// Limit loop time
		delayMicroseconds(constrain(config.SYS_LOOP_TIME - (loopEnd - loopStart), 0, config.SYS_LOOP_TIME));
	}

};

Core core;

void setup()
{
	core.Setup();
}

void loop()
{
	core.Loop();
}
