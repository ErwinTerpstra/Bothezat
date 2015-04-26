#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Arduino.h"
#include "vector3.h"

namespace bothezat
{

class Vector3;

class Config
{	
	
public:
	struct Pins
	{
		enum Pin
		{
			LED_CONTROLLER	= 23,

			I2C_SDA			= 20,
			I2C_SCL			= 21,

			RX_PWM			= 48
		};
	};

	struct Constants
	{
		/*
		 *	Radio receiver
		 */
		static const uint8_t RX_PWM_AMOUNT = 2;

		/*
		 * Motor controller
		 */
		static const uint8_t MC_MOTOR_AMOUNT = 4;

	};

	struct ChannelCalibration
	{
		uint16_t min, max, mid, deadband;

		ChannelCalibration()
		{
			min = 1050;
			mid = 1500;
			max = 1950;
			deadband = 30;
		}
	};

	struct PidConfiguration
	{
		float kp, ki, kd;
		
		PidConfiguration() : kp(1.0f), ki(1.0f), kd(1.0f)
		{

		}

		PidConfiguration(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd)
		{

		}
	};

	/*
	 * System
	 */
	uint16_t SYS_LOOP_TIME;

	/*
	 * Serial interface
	 */
	uint32_t SR_BAUD_RATE;

	ChannelCalibration RX_CHANNEL_CALIBRATION[16];

	/*
	 * Motion sensor
	 */
	uint8_t MS_CALIBRATION_SAMPLES;

	uint16_t MS_CALIBRATION_INTERVAL;

	float MS_GYRO_FILTER_RC;
	
	float MS_ACCEL_CORRECTION_RC;

	float MS_ACCEL_MAX;

	/*
	 * Flight system
	 */

	Vector3 FS_MAN_ANGULAR_VELOCITY;

	float FS_ATTI_MAX_PITCH;

	float FS_ATTI_MAX_ROLL;

	/*
	 * Motor controller
	 */
	uint32_t MC_PWM_FREQUENCY;

	uint16_t MC_PWM_PERIOD;

	uint16_t MC_PWM_MIN_COMMAND;

	uint16_t MC_PWM_MIN_OUTPUT;

	uint16_t MC_PWM_MAX_COMMAND;

	PidConfiguration MC_PID_CONFIGURATION[3];

private:
	Config();

	static Config instance;

public:
	void ReadEEPROM();

	void WriteEEPROM();

	void LoadDefaults();

public:

	static Config& Instance()
	{
		return instance;
	}


};


}


#endif