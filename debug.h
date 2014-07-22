#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "Arduino.h"
#include "module.h"

namespace bothezat
{
	
class Debug : public Module<Debug>
{
friend class Module<Debug>;

public:

private:
	char buffer[255];

protected:
	Debug()
	{

	}
	
public:
	virtual void Setup() { }
	virtual void Loop(uint32_t dt) { }

	void Print(const char *msg, va_list args)
	{
		vsprintf(buffer, msg, args);

		Serial.print(buffer);
	}

public:
	static void Print(const char *msg, ...)
	{
		va_list args;
		va_start(args, msg);

		Instance().Print(msg, args);
		va_end(args);
	}



	
};

}

#endif