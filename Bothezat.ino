// We have to include all the libraries the modules use here so that their paths get added to the compilers include path
// Arduino tool chain sucks..
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <DueFlashStorage.h>

#include "bothezat.h"

#include "i2c.h"
#include "motion_sensor.h"
#include "receiver.h"
#include "pwm_receiver.h"
#include "flight_system.h"
#include "motor_controller.h"
#include "aux_functions.h"
#include "stick_commands.h"
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
	AuxFunctions* auxFunctions;
	StickCommands* stickCommands;
	SerialInterface* serialInterface;

public:
	Core() : timer(NULL), config(Config::Instance())
	{

	}

	void Setup()
	{		
		bool configLoaded = config.ReadEEPROM();

		if (!configLoaded)
			config.LoadDefaults();

		Util::Init();
		I2C::Setup();

		// Initialize serial interface first so that we can begin sending messages
		serialInterface = &SerialInterface::Instance();
		serialInterface->Setup();

		Debug::Print("======== Bothezat ========\n");

		if (!configLoaded)
			Debug::Print("No config found, proceeding with default values\n");

		Debug::Print("Config version is: %u\n", config.VERSION);

		// Allow the user some time to settle the model
		delay(2000);

		// Initialize timer
		Timer::EnableTimers();

		// Construct other modules
		Receiver::SetReceiver(PwmReceiver::Instance());

		receiver = &Receiver::CurrentReceiver();
		motionSensor = &MotionSensor::Instance();
		flightSystem = &FlightSystem::Instance();
		auxFunctions = &AuxFunctions::Instance();
		stickCommands = &StickCommands::Instance();
		//ledController = &LedController::Instance();
		motorController = &MotorController::Instance();

		// Initialize modules
		receiver->Setup();
		motionSensor->Setup();
		flightSystem->Setup();
		auxFunctions->Setup();
		stickCommands->Setup();
		//ledController->Setup();
		motorController->Setup();

		RegisterResourceProviders();
		RegisterCommandHandlers();

		timer = Timer::GetFreeTimer();

		uint16_t precision = timer->SetPrecision(1000);
		Debug::Print("Main timer set to %d ns precision\n", precision);

		timer->Start();

		dt = 10;
		debugTime = 0;
		loopStart = lastLoopStart = timer->Micros();

		BrokenWindow();

		Debug::Print("Initialization complete!\n");
	}

	void RegisterResourceProviders()
	{
		serialInterface->RegisterResourceProvider(Page::Resource::CONFIG, 				&config);

		serialInterface->RegisterResourceProvider(Page::Resource::ORIENTATION, 			motionSensor);
		serialInterface->RegisterResourceProvider(Page::Resource::ACCEL_ORIENTATION, 	motionSensor);
		serialInterface->RegisterResourceProvider(Page::Resource::ACCELERATION, 		motionSensor);
		serialInterface->RegisterResourceProvider(Page::Resource::ANGULAR_VELOCITY, 	motionSensor);

		serialInterface->RegisterResourceProvider(Page::Resource::RECEIVER_CHANNELS, 	receiver);
		serialInterface->RegisterResourceProvider(Page::Resource::RECEIVER_NORMALIZED, 	receiver);
		serialInterface->RegisterResourceProvider(Page::Resource::RECEIVER_CONNECTED,	receiver);
		serialInterface->RegisterResourceProvider(Page::Resource::RECEIVER_CONNECTED,	receiver);
	}

	void RegisterCommandHandlers()
	{
		serialInterface->RegisterCommandHandler(Command::SAVE_CONFIG, 					&config);
		serialInterface->RegisterCommandHandler(Command::RESET_CONFIG, 					&config);
	}

	void BrokenWindow()
	{
		// Aux control functions
		AuxFunctions::ControlFunction angleModeFunction;
		angleModeFunction.type = AuxFunctions::ControlFunction::ENABLE_ANGLE_MODE;
		angleModeFunction.channel = Receiver::AUX1;
		angleModeFunction.min = 0.0f;
		angleModeFunction.max = 1.0f;

		auxFunctions->AddControlFunction(angleModeFunction);

		// Stick commands
		StickCommands::Command armCommand;
		armCommand.type = StickCommands::Command::ARM_MOTORS;
		armCommand.SetStickState(Receiver::THROTTLE, -1);
		armCommand.SetStickState(Receiver::RUDDER, -1);

		StickCommands::Command disarmCommand;
		disarmCommand.type = StickCommands::Command::DISARM_MOTORS;
		disarmCommand.SetStickState(Receiver::THROTTLE, -1);
		disarmCommand.SetStickState(Receiver::RUDDER, 1);

		stickCommands->AddCommand(armCommand);
		stickCommands->AddCommand(disarmCommand);
	}

	void Loop()
	{
		loopStart = timer->Micros();

		// Don't update deltatime on an overflow
		if (lastLoopStart <= loopStart)
			dt = loopStart - lastLoopStart;
		
		lastLoopStart = loopStart;

		receiver->Loop(dt);
		motionSensor->Loop(dt);
		//ledController->Loop(dt);
		flightSystem->Loop(dt);
		auxFunctions->Loop(dt);
		stickCommands->Loop(dt);
		motorController->Loop(dt);
		serialInterface->Loop(dt);

		#ifdef BOTH_DEBUG

		debugTime += dt;

		if (debugTime >= 1000000L)
		{
			//motionSensor->Debug();
			//receiver->Debug();
			//flightSystem->Debug();
			motorController->Debug();

			Debug::Print("Last loop time: %dus\n", dt);
			//Debug::Print("Uptime: %ds\n", timer->Micros() / 1000000);
			
			debugTime = 0;
		}

		#endif // BOTH_DEBUG

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
