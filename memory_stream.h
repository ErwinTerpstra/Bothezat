#ifndef _MEMORY_STREAM_H_
#define _MEMORY_STREAM_H_

#include "util.h"

namespace bothezat
{
	
class MemoryStream : public BinaryReadStream, public BinaryWriteStream
{

private:
	uint8_t* buffer;
	uint32_t offset;
	uint32_t length;

	bool canWrite;

public:

	MemoryStream() : buffer(NULL), length(0), offset(0), canWrite(true)
	{

	}

	MemoryStream(uint8_t* buffer, uint32_t length) : buffer(buffer), length(length), offset(0), canWrite(true)
	{

	}

	MemoryStream(const uint8_t* buffer, uint32_t length) : buffer(const_cast<uint8_t*>(buffer)), length(length), offset(0), canWrite(false)
	{

	}

	uint32_t Available() const
	{
		return length - offset;
	}

	uint32_t Offset() const
	{
		return offset;
	}

	uint32_t Length() const
	{
		return length;
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

	uint32_t Write(const uint8_t* buffer, uint32_t length)
	{
		if (!canWrite)
			return 0;
		
		length = min(length, Available());

		for (uint32_t byte = 0; byte < length; ++byte)
			this->buffer[offset + byte] = buffer[byte];

		offset += length;

		return length;
	}
	
};

}

#endif