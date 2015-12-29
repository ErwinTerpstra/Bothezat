#include "stick_commands.h"

#include "motor_controller.h"

using namespace bothezat;

StickCommands::StickCommands() : registeredCommands(0), receiver(NULL), motorController(NULL)
{
	
}

void StickCommands::Setup()
{
	receiver = &Receiver::CurrentReceiver();
	motorController = &MotorController::Instance();	
}


void StickCommands::Loop(uint32_t dt)
{
	for (uint8_t commandIdx = 0; commandIdx < registeredCommands; ++commandIdx)
	{
		Command& command = commands[commandIdx];
		
		bool sticksMatching = true;

		// Check the position of the first 4 channels
		for (uint8_t channel = 0; channel < 4; ++channel)
		{
			int desiredState = command.stickStates[channel];

			if (desiredState == 0)
				continue;

			float value = receiver->NormalizedChannel((Receiver::Channel) channel);	

			// Check if the stick is in the correct position
			if (abs(value) < MIN_STICK_POSITION || (value < 0.0f && desiredState == 1) || (value > 0.0f && desiredState == -1))
			{
				sticksMatching = false;
				break;
			}
		}

		if (sticksMatching)
		{
			if (!command.active)
			{
				ExecuteCommand(command);
				command.active = true;
			}
		}
		else
			command.active = false;
	}
}

void StickCommands::AddCommand(Command& command)
{
	if (registeredCommands >= Config::Constants::SC_MAX_COMMANDS)
		return;

	commands[registeredCommands++] = command;
}

void StickCommands::ExecuteCommand(Command& command)
{
	switch (command.type)
	{
		case Command::ARM_MOTORS:
			if (!motorController->IsArmed())
				motorController->SetArmState(true);
			break;

		case Command::DISARM_MOTORS:
			if (motorController->IsArmed())
				motorController->SetArmState(false);
			break;

		case Command::TOGGLE_ARM_STATE:
			motorController->SetArmState(!motorController->IsArmed());
			break;
	}

}
