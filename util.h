#ifndef _UTIL_H_
#define _UTIL_H_

#include "compiler.h"

namespace bothezat
{

class Util
{

public:
	typedef uint32_t crc;
	
private:

	static const uint8_t CRC_WIDTH = sizeof(crc) * 8;
	static const uint32_t CRC_TOP_BIT = 1 << (CRC_WIDTH - 1);
	static const uint32_t CRC_POLYNOMIAL = 0x04C11DB7;

	static crc crcTable[256];

public:

	static void Init()
	{
		InitCRC();
	}

	static void SwapEndianness(uint8_t* data, uint8_t size)
	{
		for (uint8_t bit = 0; bit < size / 2; ++bit)
			Swap(data + bit, data + size - bit - 1);
	}

	static void Swap(uint8_t* a, uint8_t* b)
	{
		uint8_t tmp = *a;
		*a = *b;
		*b = tmp;
	}
	
	static bool Approximately(float a, float b, float epsilon = FLT_EPSILON)
	{
		return fabs(a - b) <= epsilon;
	}

	static int Sign(int x)
	{
		return (x > 0) - (x < 0);
	}

	static float Sign(float x)
	{
		return x == 0.0f ? 0.0f : (x > 0.0f ? 1.0f : -1.0f);
	}

	static float CopySign(float a, float b)
	{
		return fabs(a) * Sign(b);
	}

	template<typename T>
	static const T& Clamp(const T& x, const T& min, const T& max)
	{
		if (x < min)
			return min;

		if (x > max)
			return max;

		return x;
	}

	static crc UpdateCRC(crc crc, uint8_t data)
	{
        data = data ^ (crc >> (CRC_WIDTH - 8));
        return crcTable[data] ^ (crc << 8);
	}

	static crc UpdateCRC(crc crc, uint16_t data)
	{
		crc = UpdateCRC(crc, static_cast<uint8_t>((data >> 8) & 0xFF));
		crc = UpdateCRC(crc, static_cast<uint8_t>(data & 0xFF));

		return crc;
	}

	static crc UpdateCRC(crc crc, uint32_t data)
	{
		crc = UpdateCRC(crc, static_cast<uint8_t>((data >> 24) & 0xFF));
		crc = UpdateCRC(crc, static_cast<uint8_t>((data >> 16) & 0xFF));
		crc = UpdateCRC(crc, static_cast<uint8_t>((data >> 8) & 0xFF));
		crc = UpdateCRC(crc, static_cast<uint8_t>(data & 0xFF));
		
		return crc;
	}

	static crc CalculateCRC(const uint8_t* message, uint32_t messageLength)
	{
	    uint8_t data;
	    crc remainder = 0;

	    // Divide the message by the polynomial, a byte at a time.
	    for (uint32_t byte = 0; byte < messageLength; ++byte)
	    {
	        data = message[byte] ^ (remainder >> (CRC_WIDTH - 8));
	        remainder = crcTable[data] ^ (remainder << 8);
	    }

	    // The final remainder is the CRC.
	    return remainder;

	}

private:

	static void InitCRC()
	{
		crc remainder;

	    // Compute the remainder of each possible dividend.
	    for (uint32_t dividend = 0; dividend < 256; ++dividend)
	    {
	        // Start with the dividend followed by zeros.
	        remainder = dividend << (CRC_WIDTH - 8);

	        // Perform modulo-2 division, a bit at a time.
	        for (uint8_t bit = 8; bit > 0; --bit)
	        {
	            // Try to divide the current data bit.			
	            if (remainder & CRC_TOP_BIT)
	            {
	                remainder = (remainder << 1) ^ CRC_POLYNOMIAL;
	            }
	            else
	            {
	                remainder = (remainder << 1);
	            }
	        }

	        // Store the result into the table.
	        crcTable[dividend] = remainder;
	    }
	}

};
}

#endif