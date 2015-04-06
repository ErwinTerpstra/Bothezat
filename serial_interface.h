#ifndef _SERIAL_INTERFACE_H_
#define _SERIAL_INTERFACE_H_

#include "Arduino.h"
#include "bothezat.h"

#include "module.h"

#include "ring_buffer.h"
#include "arduino_serial.h"
 
namespace bothezat
{

class SerialInterface : public Module<SerialInterface>
{
friend class Module<SerialInterface>;

private:
	static const uint32_t READ_CHUNK_SIZE = 16;

	static const uint32_t MESSAGE_BUFFER_SIZE	= 128;
	static const uint32_t RESOURCE_BUFFER_SIZE 	= 256;

	struct Message : public Serializable, public Deserializable
	{
		// Each message starts with this number to identify message boundaries
		static const uint32_t MESSAGE_MAGIC = 0xB074E6A7;

		// The size of a complete message, with a zero length payload
		static const uint32_t HEADER_SIZE = 16;

		static const uint32_t MAX_PAYLOAD_LENGTH = 1024 * 4;

		enum Phase 
		{
			PHASE_REQUEST 		= 0x00,
			PHASE_RESPONSE 		= 0x01,

			PHASE_LAST_VALUE	= ENUM_PADDING_VALUE
		};

		enum Type
		{
			TYPE_PAGE  			= 0x01,
			TYPE_COMMAND 		= 0x02,
			TYPE_LOG 			= 0x03,

			TYPE_LAST_VALUE 	= ENUM_PADDING_VALUE
		};

		// A magic number used to sync the stream to the start of a message, should have the value in MESSAGE_MAGIC
		uint32_t magic;

		// The phase for this message, this is used to distinguish between request and response messages
		Phase phase;
	
		// The message type
		Type type;

		// The unique ID for this message, for response message this should be the ID of the corresponding request message
		uint32_t id;

		// The total length of the payload, in bytes
		uint16_t payloadLength;

		// The CRC over the message header, includes all the fields in the message struct up until this
		Util::crc crc;

		const uint8_t* payload;

		bool headersRead;

		Message() : magic(MESSAGE_MAGIC), crc(0), payload(NULL), headersRead(false)
		{

		}

		void Serialize(BinaryWriteStream& stream) const
		{
			// Send the message header data 
			stream.Write(magic);
			stream.Write(phase);
			stream.Write(type);
			stream.Write(id);
			stream.Write(payloadLength);
			stream.Write(crc);
		}

		uint32_t SerializedSize() const
		{
			return sizeof(magic) + sizeof(phase) + sizeof(type) + sizeof(id) + sizeof(payloadLength) + sizeof(crc);
		}

		bool Deserialize(BinaryReadStream& stream)
		{
			// Check if we have enough data to read the message headers
			if (stream.Available() < HEADER_SIZE)
				return false;

			// Read all the fields into the struct
			magic  			= stream.ReadUInt32();
			crc				= stream.ReadUInt32();
			phase 			= (Message::Phase) stream.ReadUInt32();
			type 			= (Message::Type) stream.ReadUInt32();
			id 				= stream.ReadUInt32();
			payloadLength 	= stream.ReadUInt16();

			return true;
		}
	};

	struct Page
	{
		static const uint32_t MAX_RESOURCES_PER_PAGE = 32;

		struct Resource : public Serializable
		{
			enum Type
			{
				ORIENTATION 			= 0x01,
				ACCEL_ORIENTATION 		= 0x02,

				INVALID_RESOURCE 		= 0xFF,

				LAST_VALUE 				= ENUM_PADDING_VALUE
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
				return sizeof(type) + sizeof(length) + length;
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
				if (stream.Available() < numResources);
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

	struct Command
	{
		enum Type 
		{
			CALIBRATE 				= 0x01,
			SET_RESOURCE_VALUE 		= 0x02,

			LAST_VALUE 				= ENUM_PADDING_VALUE
		};

		enum State
		{
			COMMAND_UNKOWN,
			COMMAND_OK,
			COMMAND_ERROR
		};

		struct RequestMessage
		{
			Type command;
		};

		struct ResponseMessage
		{
			State state;
		};

	};

	// Intermediate buffer for reading serial data. Data is read to this buffer and then transferred to the larger ring buffer
	uint8_t readChunk[READ_CHUNK_SIZE];

	RingBuffer messageBuffer;
	RingBuffer resourceBuffer;

	// The message that was last received, or is currently being read
	Message lastReceivedMessage;

	// Shared buffer for message payload, is allocated with MAX_PAYLOAD_LENGTH
	uint8_t* payloadBuffer;

	// The serial port wrapper to use for all communication
	SerialPort serialPort;

protected:
	SerialInterface();

public:
	~SerialInterface();

	virtual void Setup();
	virtual void Loop(uint32_t dt);

	void SendLog(const char* msg);

private:

	void ReadMessages();
	uint32_t ProcessMessages();
	void ProcessMessage(const Message& message);

	void ProcessPageRequest(const Message& requestMessage);
	void ProcessCommandRequest(const Message& requestMessage);
	void ProcessLogRequest(const Message& requestMessage);

	bool ReadMessage(Message& message);
	bool ReadMessageHeaders(Message& message);

	void SendMessage(Message& message);
	void SendResponseMessage(const Message& requestMessage, Serializable& payload);
	
	void Send(uint8_t data);
	void Send(const uint8_t* buffer, uint32_t size);

	bool SyncMessageStream();
	void PurgeMessageBuffer();

	Util::crc CalculateMessageCRC(const Message& message) const;
	uint32_t GetNextMessageID() { return random(0, 1 << 31); };

};


}

#endif