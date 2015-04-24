#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "Arduino.h"

namespace bothezat
{
	
class Debug
{
public:
	static const uint32_t MAX_DEBUG_PRINT_LENGTH = 512;

private:
	char buffer[MAX_DEBUG_PRINT_LENGTH];

	Debug();
	
public:
	void Print(const char* msg, va_list args);

public:

	static void Print(const char* msg, ...);

	static bool AssertHandler(const char* code, const char* file, const uint32_t line);

	static bool Halt();
	
	static Debug& Instance()
	{
		static Debug instance;

		return instance;
	}
};

#define assert(x) ((void)(!(x) && Debug::AssertHandler(#x, __FILE__, __LINE__) && Debug::Halt()))
//#define assert(x) 

}

#endif