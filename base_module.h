#ifndef _BASE_MODULE_H_
#define _BASE_MODULE_H_

#include "Arduino.h"

namespace bothezat
{
	
class BaseModule
{

protected:
	BaseModule() {  }
	
public:

	virtual void Setup() = 0;
	virtual void Loop(uint32_t dt) = 0;
	virtual void Debug() const { };	
};

}

#endif