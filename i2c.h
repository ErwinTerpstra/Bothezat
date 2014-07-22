#ifndef _I2C_H_
#define _I2C_H_

#include "Arduino.h"

#include <Wire.h>

namespace bothezat
{
	
/*
 * Utility class for I2C communication. Is a wrapper around the Arduino Wire library
 */
class I2C
{


public:
	enum ErrorCodes
	{
		ERR_OK				= 0,

		ERR_REG_SELECT_FAILED		= 10,

		ERR_BUFFER_OVERFLOW			= 11,
		ERR_NACK_ADDRESS			= 12,
		ERR_NACK_DATA				= 13,
		ERR_UNKOWN_ERROR			= 14,

		ERR_DEVICE_ABORTED 			= 15,


		ERR_END_TRANSMISION_BASE	= 10,
	};

public:


	static void Setup()
	{
		ResetBus();
		Wire.begin();
	}

	/*
	 * Reads data from a device register into a buffer
	 */
	static int Read(uint8_t address, uint8_t start, uint8_t* buffer, uint8_t size)
	{
		uint8_t error;

		Wire.beginTransmission(address);

		// Write the register address
		if (Wire.write(start) != 1)
			return ERR_REG_SELECT_FAILED;

		// End the write transmission but hold the bus
		error = Wire.endTransmission(false);

		if (error != 0)
			return ERR_END_TRANSMISION_BASE + error;	// Convert error to enum value

		Wire.requestFrom(address, size, true);

		// Read desired amount of bytes
		uint16_t bytesRead = 0;
		while (Wire.available() && bytesRead < size)
			buffer[bytesRead++] = Wire.read();

		// Check if complete data was available
		if (bytesRead < size)
			return ERR_DEVICE_ABORTED;

		return ERR_OK;
	}

	/*
	 * Writes data to a device register from a buffer
	 */
	static int Write(uint8_t address, uint8_t start, const uint8_t* data, uint8_t size)
	{
		uint8_t bytesWritten;

		Wire.beginTransmission(address);

		// Write the register address
		bytesWritten = Wire.write(start);
		if (bytesWritten != 1)
			return ERR_REG_SELECT_FAILED;

		// Write the data buffer
		bytesWritten = Wire.write(data, size);
		if (bytesWritten != size)
			return ERR_DEVICE_ABORTED;

		// End the write transmission and release the bus
		uint8_t error = Wire.endTransmission(false);

		if (error != 0)
			return ERR_END_TRANSMISION_BASE - error;	// Convert error to enum value

		return ERR_OK;
	}

	/*
	 * Convenience method to write a single byte to a single register
	 */
	static int WriteRegister(uint8_t address, uint8_t reg, uint8_t data)
	{
		return Write(address, reg, &data, 1);
	}

	/*
	 * Resets the I2C bus by forcing a pulse to the clock pin
	 */
	static void ResetBus()
	{
		pinMode(PIN_WIRE_SCL, OUTPUT);
		bool busStuck;

		do
		{
			pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
			delayMicroseconds(5);

			busStuck = false;

			// I2C bus stuck, try clocking through all data a slave still wants to send
			while (digitalRead(PIN_WIRE_SDA) == LOW)
			{
				busStuck = true;

				digitalWrite(PIN_WIRE_SCL, HIGH);	delayMicroseconds(5);
				digitalWrite(PIN_WIRE_SCL, LOW);	delayMicroseconds(5);
				digitalWrite(PIN_WIRE_SCL, HIGH);	delayMicroseconds(5);
			}

			if (busStuck)
			{
				pinMode(PIN_WIRE_SDA, OUTPUT);
				delayMicroseconds(5);

				// Generate a stop condition
				digitalWrite(PIN_WIRE_SCL, HIGH);
				digitalWrite(PIN_WIRE_SDA, LOW);
				delayMicroseconds(5);
				digitalWrite(PIN_WIRE_SDA, HIGH);
				delayMicroseconds(5);

				digitalWrite(PIN_WIRE_SCL, HIGH);	delayMicroseconds(5);
				digitalWrite(PIN_WIRE_SCL, LOW);	delayMicroseconds(5);
				digitalWrite(PIN_WIRE_SCL, HIGH);	delayMicroseconds(5);
			}

		} while (busStuck);
	}

};
	

}

#endif