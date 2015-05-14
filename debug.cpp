#include "Arduino.h"

#include "debug.h"

#include "serial_interface.h"

using namespace bothezat;

Debug::Debug()
{
	
}

void Debug::Print(const char* msg, va_list args)
{
	vsprintf(buffer, msg, args);

	SerialInterface::Instance().SendLog(buffer);
	
	//Serial.print(buffer);
}

void Debug::Print(const char* msg, ...)
{
	va_list args;
	va_start(args, msg);

	Instance().Print(msg, args);
	va_end(args);
}

bool Debug::AssertHandler(const char* code, const char* file, const uint32_t line)
{
	Print("Assert failed!\n%s at %s:%d\n", code, file, line);
	return true;
}

bool Debug::Halt()
{
	#ifdef BOTH_DEBUG
		while(1);
	#endif

	return true;
}