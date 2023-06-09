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

	static const uint32_t MESSAGE_BUFFER_SIZE	= 1024 * 4;
	static const uint32_t DATA_BUFFER_SIZE 		= 1024 * 4;

	struct Message : public Serializable, public Deserializable
	{
		// Each message starts with this number to identify message boundaries
		static const uint32_t MESSAGE_MAGIC = 0xB074E6A7;

		// The size of a complete message, with a zero length payload
		static const uint32_t HEADER_SIZE = 22;

		static const uint32_t MAX_PAYLOAD_LENGTH = 1024 * 4;

		enum Phase 
		{
			PHASE_REQUEST 		= 0x00,
			PHASE_RESPONSE 		= 0x01,

			PHASE_LAST_VALUE	= 0xFF
		};

		enum Type
		{
			TYPE_PAGE  			= 0x01,
			TYPE_COMMAND 		= 0x02,
			TYPE_LOG 			= 0x03,

			TYPE_LAST_VALUE 	= 0xFF
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
			stream.Write(crc);
			stream.Write((uint8_t) phase);
			stream.Write((uint8_t) type);
			stream.Write(id);
			stream.Write(payloadLength);
		}

		uint32_t SerializedSize() const
		{
			return sizeof(magic) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(id) + sizeof(payloadLength) + sizeof(crc);
		}

		bool Deserialize(BinaryReadStream& stream)
		{
			// Check if we have enough data to read the message headers
			if (stream.Available() < HEADER_SIZE)
				return false;

			// Read all the fields into the struct
			magic  			= stream.ReadUInt32();
			crc				= stream.ReadUInt32();
			phase 			= (Message::Phase) stream.ReadByte();
			type 			= (Message::Type) stream.ReadByte();
			id 				= stream.ReadUInt32();
			payloadLength 	= stream.ReadUInt16();

			return true;
		}
	};

	// Intermediate buffer for reading serial data. Data is read to this buffer and then transferred to the larger ring buffer
	uint8_t readChunk[READ_CHUNK_SIZE];

	ResourceProvider* resourceProviders[256];
	CommandHandler* commandHandlers[256];

	RingBuffer messageBuffer;
	RingBuffer dataBuffer;

	// The message that was last received, or is currently being read
	Message lastReceivedMessage;

	// Shared buffer for message payload, is allocated with MAX_PAYLOAD_LENGTH
	uint8_t* payloadBuffer;

	// The serial port wrapper to use for all communication
	SerialPort serialPort;

	// Whether the headers have been read for the message that we currently are receiving
	bool headersRead;

protected:
	SerialInterface();

public:
	~SerialInterface();

	virtual void Setup();
	virtual void Loop(uint32_t dt);

	void RegisterResourceProvider(Page::Resource::Type type, ResourceProvider* provider);
	void RegisterCommandHandler(Command::Type type, CommandHandler* handler);

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