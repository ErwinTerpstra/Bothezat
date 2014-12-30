#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include "binary_stream.h"

namespace bothezat
{
	
class RingBuffer
{
public:
	class ReadStream;
	class WriteStream;
	
	friend class ReadStream;
	friend class WriteStream;

	class ReadStream : public BinaryReadStream
	{
	friend class RingBuffer;

	private:
		const RingBuffer& buffer;

		uint32_t offset;

		ReadStream(const RingBuffer& buffer) : buffer(buffer), offset(buffer.origin)
		{

		}
	public:

		void Reset()
		{
			offset = buffer.origin;
		}

		uint32_t Available() const
		{
			return (buffer.size + (buffer.writeStream.offset - offset)) % buffer.size;
		}

		uint32_t Seek(int32_t amount)
		{
			assert(amount >= 0 && "RingBuffer stream can only seek forward");

			amount = min(amount, Available());
			
			offset += amount;
			offset = offset % buffer.size;

			return amount;
		}

		uint32_t Read(uint8_t* buffer, uint32_t length, bool peek = false)
		{
			uint32_t bytesRead = 0;
			uint32_t offset = this->offset;

			// Keep reading until we caught up with the buffer or enough bytes have been read
			while (offset != this->buffer.writeStream.offset && bytesRead <= length)
			{
				// Copy the next byte from the internal buffer to our output buffer
				buffer[bytesRead] = this->buffer.data[offset];
				++bytesRead;

				// Wrap read offset around if it reaches the end of the buffer
				if (++offset >= this->buffer.size)
					offset = 0;
			}

			// If we want to consume the data, update the internal reading pointer
			if (!peek)
				this->offset = offset;

			return bytesRead;
		}
	};

	class WriteStream : public BinaryWriteStream
	{
	friend class RingBuffer;

	private:
		const RingBuffer& buffer;

		uint32_t offset;

		WriteStream(const RingBuffer& buffer) : buffer(buffer), offset(buffer.origin)
		{

		}

	public:

		void Reset()
		{
			offset = buffer.origin;
		}

		const uint8_t* DataPointer() const
		{
			return buffer.data + offset;
		}

		uint32_t Write(const uint8_t* buffer, uint32_t length)
		{
			uint32_t bytesWritten = 0;

			while (bytesWritten <= length)
			{
				this->buffer.data[offset] = buffer[bytesWritten];
				++bytesWritten;

				// Wrap write offset around if it reaches the end of the buffer
				if (++offset >= this->buffer.size)
					offset = 0;

				// If the write offset is equal to the origin point AFTER incrimation that means the buffer is full
				// Stop writing so that the user knows to read the buffer first
				if (offset == this->buffer.origin)
					break;
			}

			return bytesWritten;
		};

	};

private:
	uint8_t* data;
	uint32_t size;

	uint32_t origin;

public:
	ReadStream readStream;
	WriteStream writeStream;

public:

	RingBuffer() : data(NULL), size(0), readStream(*this), writeStream(*this)
	{

	}

	~RingBuffer()
	{
		if (data != NULL)
		{
			free(data);
			data = NULL;
		}
	}

	void Allocate(uint32_t size)
	{
		this->size = size;
		data = static_cast<uint8_t*>(malloc(size));

		Clear();
	}

	uint32_t UsedBytes()
	{
		return (size + (writeStream.offset - origin)) % size;
	}

	uint32_t FreeBytes()
	{
		return size - UsedBytes();
	}

	void Trim()
	{
		origin = readStream.offset;
	}

	void Clear()
	{
		origin = 0;
		
		readStream.Reset();
		writeStream.Reset();
	}
	
};


}


#endif