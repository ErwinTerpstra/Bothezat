#ifndef _CONFIG_H_
#define _CONFIG_H_

namespace bothezat
{

class PinConfig
{	
public:
	enum Pin
	{
		LED_CONTROLLER	= 23,

		I2C_SDA			= 20,
		I2C_SCL			= 21,

		RX_PWM			= 50
	};

	static const int RX_PWM_AMOUNT = 4;
	static const int MOTOR_AMOUNT = 4;

};


}


#endif