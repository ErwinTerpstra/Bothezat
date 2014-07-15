#include "config.h"
#include "led_controller.h"

#include <Adafruit_NeoPixel.h>

using namespace bothezat;

LedController::LedController() : output(8, PinConfig::LED_CONTROLLER, NEO_GRB + NEO_KHZ800), shift(0)
{

}

void LedController::Setup()
{
	output.begin();
 
	output.setBrightness(20);   
	output.show();
	
}

void LedController::Loop(uint32_t dt)
{
	uint8_t numPixels = output.numPixels();

	++shift;
    output.setPixelColor(shift++ % numPixels, 255, 0, 0);
    output.setPixelColor(shift++ % numPixels, 255, 255, 0);
    output.setPixelColor(shift++ % numPixels, 0, 255, 0);
    output.setPixelColor(shift++ % numPixels, 0, 255, 255);
    output.setPixelColor(shift++ % numPixels, 0, 0, 255);
    output.setPixelColor(shift++ % numPixels, 255, 0, 255);
    output.setPixelColor(shift++ % numPixels, 255, 255, 255);
    output.setPixelColor(shift++ % numPixels, 255, 255, 255); 

	output.show();
}