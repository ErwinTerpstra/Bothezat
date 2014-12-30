#ifndef _MEMORY_STREAM_H_
#define _MEMORY_STREAM_H_

#include "util.h"

namespace bothezat
{
	
class MemoryStream : public BinaryReadStream
{

private:
	const uint8_t* buffer;
	uint32_t offset;
	uint32_t size;

public:

	MemoryStream(const uint8_t* buffer, uint32_t size) : buffer(buffer), size(size), offset(0)
	{

	}

	uint32_t Available() const
	{
		return size - offset;
	}

	uint32_t Seek(int32_t amount)
	{
		// Limit the amount of seeking to both the start and the end of the buffer
		amount = Util::Clamp(amount, static_cast<int32_t>(-offset), static_cast<int32_t>(Available()));

		offset += amount;

		return amount;
	}

	uint32_t Read(uint8_t* buffer, uint32_t length, bool peek = false)
	{
		length = min(length, Available());

		for (uint32_t byte = 0; byte < length; ++byte)
			buffer[byte] = this->buffer[offset + byte];

		if (!peek)
			offset += length;

		return length;
	}

	/*
	uint32_t Write(const uint8_t* buffer, uint32_t length)
	{
		length = min(length, Available());

		for (uint32_t byte = 0; byte < length; ++byte)
			this->buffer[offset + byte] = buffer[byte];

		offset += length;

		return length;
	}
	*/
	
};

}

#endif