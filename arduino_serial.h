#ifndef _ARDUINO_SERIAL_H_
#define _ARDUINO_SERIAL_H_

#include "Arduino.h"

#include "binary_stream.h"

namespace bothezat
{

class SerialPort : public BinaryStream
{

private:
	// The arduino platform serial port
	HardwareSerial& serial;

public:
	SerialPort(HardwareSerial& serial) : serial(serial)
	{

	}

	void Begin(uint32_t baudRate)
	{
		serial.begin(baudRate);
	}

	void End()
	{
		serial.end();
	}
	
	uint32_t Available() const
	{
		return serial.available();
	}

	uint32_t Seek(int32_t amount)
	{
		assert(amount >= 0 && "Serial stream can only seek forward");

		while (amount > 0)
		{
			serial.read();
			--amount;
		}
	}

	uint32_t Read(uint8_t* buffer, uint32_t length, bool peek = false)
	{
		assert(!peek && "Peek reading not supported on Arduino serial port");
		return serial.readBytes(buffer, length);
	}

	uint32_t Write(const uint8_t* buffer, uint32_t size)
	{
		return serial.write(buffer, size);
	}

	static SerialPort& Instance()
	{
		static SerialPort instance(Serial);

		return instance;
	}
};

}

#endif