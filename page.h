#ifndef _PAGE_H_
#define _PAGE_H_

#include "binary_stream.h"

namespace bothezat
{
	
struct Page
{
	static const uint32_t MAX_RESOURCES_PER_PAGE = 32;

	struct Resource : public Serializable
	{
		enum Type
		{
			// System
			CONFIG 					= 0x01,
			
			// Motion sensor
			ORIENTATION 			= 0x10,
			ACCEL_ORIENTATION 		= 0x11,
            ACCELERATION            = 0x15,
            ANGULAR_VELOCITY        = 0x16,

            // Motor controller
            YAW_PID                 = 0x20,
            PITCH_PID               = 0x21,
            ROLL_PID                = 0x22,
            YAW_PID_DEBUG           = 0x23,
            PITCH_PID_DEBUG         = 0x24,
            ROLL_PID_DEBUG          = 0x25,
            MOTOR_OUTPUT 			= 0x26,

            // Receiver
            RECEIVER_CHANNELS		= 0x30,
            RECEIVER_NORMALIZED 	= 0x31,
            RECEIVER_CONNECTED 		= 0x32,

			INVALID_RESOURCE 		= 0xFF,
		};

		Type type;

		uint32_t length;

		const uint8_t* data;

		Resource() : type(INVALID_RESOURCE), length(0), data(NULL)
		{

		}

		void Serialize(BinaryWriteStream& stream) const
		{
			stream.Write(static_cast<uint8_t>(type));
			stream.Write(length);
			stream.Write(data, length);
		}

		uint32_t SerializedSize() const
		{
			return sizeof(uint8_t) + sizeof(length) + length;
		}
	};

	struct RequestMessage : public Deserializable
	{
		uint32_t numResources;

		Resource::Type resources[MAX_RESOURCES_PER_PAGE];

		bool Deserialize(BinaryReadStream& stream)
		{
			// Make sure there is enough data to start in the stream
			if (stream.Available() < sizeof(numResources))
				return false;

			numResources = stream.ReadUInt32();

			// Make sure there is enough data to read all resource types
			if (stream.Available() < numResources)
				return false;

			// Read all resource types
			for (uint32_t resourceIdx = 0; resourceIdx < numResources; ++resourceIdx)
				resources[resourceIdx] = static_cast<Resource::Type>(stream.ReadByte());

			return true;
		}
	};

	struct ResponseMessage : public Serializable
	{
		uint32_t numResources;

		Resource resources[MAX_RESOURCES_PER_PAGE];

		void Serialize(BinaryWriteStream& stream) const
		{
			stream.Write(numResources);

			// Serialize all the resources to the stream
			for (uint32_t resourceIdx = 0; resourceIdx < numResources; ++resourceIdx)
				resources[resourceIdx].Serialize(stream);
		}

		uint32_t SerializedSize() const
		{
			uint32_t size = sizeof(numResources);

			for (uint32_t resourceIdx = 0; resourceIdx < numResources; ++resourceIdx)
				size += resources[resourceIdx].SerializedSize();

			return size;
		}
	};

};

class ResourceProvider
{
public:
	virtual uint16_t SerializeResource(Page::Resource::Type type, BinaryWriteStream& stream);

};

class SerializableResource : public ResourceProvider
{

public:
	const Page::Resource::Type type;

	Serializable& serializable;

public:
	SerializableResource(Page::Resource::Type type, Serializable& serializable) :
		type(type), serializable(serializable)
	{

	}


	uint16_t SerializeResource(Page::Resource::Type type, BinaryWriteStream& stream)
	{
		if (type != this->type)
			return 0;

		serializable.Serialize(stream);

		return serializable.SerializedSize();
	}

};


}

#endif