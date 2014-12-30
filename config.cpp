#include "config.h"

using namespace bothezat;

Config Config::instance;

Config::Config()
{
	LoadDefaults();
}

// TODO: implement this shit
void Config::ReadEEPROM()
{

}

void Config::WriteEEPROM()
{

}

void Config::LoadDefaults()
{

	/*
	 * System
	 */
	SYS_LOOP_TIME				= 0;			// Minimum loop time (us) the system is trying to match, zero means as fast as possible

	/*
	 * Serial interface
	 */
	SR_BAUD_RATE 				= 115200; 		// Baud rate for serial communication

	/*
	 * Radio receiver
	 */
	RX_SIGNAL_MID				= 1500;			// The signal the receiver sees as neutral

	/*
	 * Motion sensor
	 */
	MS_CALIBRATION_SAMPLES		= 10;			// Amount of samples for IMU
	MS_CALIBRATION_INTERVAL		= 100;			// Time between IMU calibration samples
	MS_GYRO_FILTER_RC			= 2.5f;			// RC for gyro high pass filter
	MS_ACCEL_CORRECTION_RC		= 0.001f;		// Lower means slower correction to gyro by accelerometer
	MS_ACCEL_MAX				= 0.15f;		// Accelerometer values with a larger deviation from 1G than this will get discarded

	/*
	 * Flight system
	 */

	FS_MAN_ANGULAR_VELOCITY		= Vector3(50.0f, 50.0f, 50.0f);	// Angular velocity at max stick input for manual mode
	FS_ATTI_MAX_PITCH			= 45.0f;						// Pitch angle at max stick input for atti mode
	FS_ATTI_MAX_ROLL			= 45.0f;						// Roll angle at max stick input for atti mode

	/*
	 * Motor controller
	 */
	MC_PWM_FREQUENCY			= 488;
	MC_PWM_MIN_COMMAND			= 950;
	MC_PWM_MAX_COMMAND			= 2050;
}