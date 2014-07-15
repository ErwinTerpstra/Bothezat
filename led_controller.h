#ifndef _LED_CONTROLLER_H_
#define _LED_CONTROLLER_H_

#include "Arduino.h"

#include "module.h"

#include <Adafruit_NeoPixel.h>

namespace bothezat
{

class LedController : public Module<LedController>
{
friend class Module<LedController>;

private:
	Adafruit_NeoPixel output;

	uint8_t shift;

protected:
	LedController();

public:

	virtual void Setup();

	virtual void Loop();

};

}

#endif