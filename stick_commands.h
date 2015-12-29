#ifndef _STICK_COMMANDS_H_
#define _STICK_COMMANDS_H_

#include "Arduino.h"
#include "bothezat.h"
#include "module.h"

#include "receiver.h"

namespace bothezat
{

class MotorController;

class StickCommands : public Module<StickCommands>
{
friend class Module<StickCommands>;

private:
	static const float MIN_STICK_POSITION = 0.85f;

public:

	struct Command
	{
		enum Type
		{
			UNKNOWN,

			ARM_MOTORS,
			DISARM_MOTORS,
			TOGGLE_ARM_STATE
		};

		Type type;

		int8_t stickStates[4];

		bool active;

		Command() : type(UNKNOWN), active(false)
		{
			for (uint8_t channel = 0; channel < 4; ++channel)
				stickStates[channel] = 0;
		}

		void SetStickState(Receiver::Channel channel, int8_t state)
		{
			stickStates[channel] = state;
		}
	};

	Command commands[Config::Constants::SC_MAX_COMMANDS];

	uint8_t registeredCommands;

	Receiver* receiver;

	MotorController* motorController;

protected:
	StickCommands();

public:
	virtual void Setup();

	virtual void Loop(uint32_t dt);

	void AddCommand(Command& function);

private:

	void ExecuteCommand(Command& command);

};

}


#endif