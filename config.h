#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Arduino.h"

namespace bothezat
{

struct Config
{	
	struct Pins
	{
		enum Pin
		{
			LED_CONTROLLER	= 23,

			I2C_SDA			= 20,
			I2C_SCL			= 21,

			RX_PWM			= 50
		};
	};

	/*
	 *	Radio receiver
	 */
	static const uint8_t RX_PWM_AMOUNT = 4;

	/*
	 * Motion sensor
	 */
	static const uint8_t MS_CALIBRATION_SAMPLES = 10;		// Amount of samples for IMU

	static const uint16_t MS_CALIBRATION_INTERVAL = 100;	// Time between IMU calibration samples

	static const float MS_GYRO_FILTER_RC = 2.5f;			// RC for gyro high pass filter
	
	static const float MS_ACCEL_CORRECTION_RC = 0.000010f;	// Lower means slower correction to gyro by accelerometer

	static const float MS_ACCEL_MAX = 0.15f;				// Accelerometer values with a larger deviation from 1G than this will get discarded

	/*
	 * Motor controller
	 */
	static const uint8_t MC_MOTOR_AMOUNT = 4;

};


}


#endif