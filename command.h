#ifndef _COMMAND_H_
#define _COMMAND_H_

#include "binary_stream.h"
#include "ring_buffer.h"

namespace bothezat
{

struct Command 
{
	enum Type 
	{
		SAVE_CONFIG				= 0x01,
		RESET_CONFIG 			= 0x02,

		CALIBRATE_ACCELEROMETER	= 0x10,

		INVALID_COMMAND			= 0xFF
	};

	enum State
	{
		COMMAND_OK 				= 0x00,
		COMMAND_UNKOWN 			= 0x01,
		COMMAND_ERROR			= 0x02
	};

	struct RequestMessage : public Deserializable
	{
		Type type;

		uint32_t length;

		RingBuffer& buffer;

		RequestMessage(RingBuffer& buffer) : buffer(buffer)
		{

		}

		bool Deserialize(BinaryReadStream& stream)
		{
			// Check if there is enough data to read the type and size
			if (stream.Available() < sizeof(uint8_t) + sizeof(uint32_t))
				return false;

			type = static_cast<Type>(stream.ReadByte());
			length = stream.ReadUInt32();

			// Check if there is enough data to read the payload
			if (stream.Available() < length)
				return false;

			stream.ReadTo(buffer.writeStream, length);

			return true;
		}
	};

	struct ResponseMessage : public Serializable
	{
		State state;

		void Serialize(BinaryWriteStream& stream) const
		{
			stream.Write((uint8_t) state);
		}

		uint32_t SerializedSize() const
		{
			return sizeof(uint8_t);
		}
	};

};

class CommandHandler
{
public:
	virtual bool HandleCommand(Command::RequestMessage& command);

};

}

#endif