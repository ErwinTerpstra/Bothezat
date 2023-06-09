#include "Arduino.h"
#include "bothezat.h"

#include "serial_interface.h"
#include "memory_stream.h"

#include "motion_sensor.h"

#include "command.h"

using namespace bothezat;

SerialInterface::SerialInterface() : payloadBuffer(NULL), serialPort(SerialPort::Instance())
{
	// Initialize all resource providers and command handlers to NULL
	for (uint16_t idx = 0; idx < 256; ++idx)
	{
		resourceProviders[idx] = NULL;
		commandHandlers[idx] = NULL;
	}
}

SerialInterface::~SerialInterface()
{
	if (payloadBuffer != NULL)
	{
		free(payloadBuffer);
		payloadBuffer = NULL;
	}
}

void SerialInterface::Setup()
{
	messageBuffer.Allocate(MESSAGE_BUFFER_SIZE);
	dataBuffer.Allocate(DATA_BUFFER_SIZE);

	payloadBuffer = static_cast<uint8_t*>(malloc(Message::MAX_PAYLOAD_LENGTH));

	serialPort.Begin(config.SR_BAUD_RATE);
}

void SerialInterface::Loop(uint32_t dt)
{
	ReadMessages();
	ProcessMessages();
}

void SerialInterface::RegisterResourceProvider(Page::Resource::Type type, ResourceProvider* provider)
{
	resourceProviders[type] = provider;
}

void SerialInterface::RegisterCommandHandler(Command::Type type, CommandHandler* handler)
{
	commandHandlers[type] = handler;
}

void SerialInterface::SendLog(const char* msg)
{
	Message message;
	message.phase 			= Message::PHASE_REQUEST;
	message.type 			= Message::TYPE_LOG;
	message.payloadLength 	= strlen(msg);
	message.payload 		= reinterpret_cast<const uint8_t*>(msg);

	SendMessage(message);
}

void SerialInterface::ReadMessages()
{
	// Read all the data currently in the serial buffer
	while (serialPort.Available() > 0)
	{
		// Read in chunks of a maximum of READ_CHUNK_SIZE
		uint32_t chunkSize = min(Serial.available(), READ_CHUNK_SIZE);
		chunkSize = serialPort.Read(readChunk, chunkSize);

		// If there is not enough room for the data in the buffer we try to process the messages that are currently in the buffer
		// If no messages could be processed this means that the current message is too big for the message buffer
		// Clear the buffer, discarding the data for the culprit message
		if (messageBuffer.FreeBytes() < chunkSize && ProcessMessages() == 0)
			PurgeMessageBuffer();

		// Attempt to transfer the newly read data to the message buffer
		uint32_t bytesWritten = messageBuffer.writeStream.Write(readChunk, chunkSize);
		assert(bytesWritten == chunkSize);

	}
}

uint32_t SerialInterface::ProcessMessages()
{
	uint32_t messagesProcessed = 0;

	while (ReadMessage(lastReceivedMessage))
	{
		ProcessMessage(lastReceivedMessage);
		++messagesProcessed;

		headersRead = false;
	}

	return messagesProcessed;
}

void SerialInterface::ProcessMessage(const Message& message)
{
	// For now, FC only handles requests
	if (message.phase != Message::PHASE_REQUEST)
		return;

	switch (message.type)
	{
		case Message::TYPE_PAGE:
			ProcessPageRequest(message);
			break;

		case Message::TYPE_COMMAND:
			ProcessCommandRequest(message);
			break;

		case Message::TYPE_LOG:
			ProcessLogRequest(message);
			break;
	}
}

void SerialInterface::ProcessPageRequest(const Message& requestMessage)
{
	dataBuffer.Clear();

	Page::RequestMessage request;
	Page::ResponseMessage response;

	// Deserialize the request struct from the payload
	MemoryStream stream(requestMessage.payload, requestMessage.payloadLength);
	bool result = request.Deserialize(stream);

	if (!result)
	{
		Debug::Print("Failed to deserialize page request!\n");
		return;
	}

	// We always respond with the same number of resources, even if some of them are invalid
	response.numResources = request.numResources;

	// Iterate through each resource in the request 
	for (uint32_t resourceIdx = 0; resourceIdx < request.numResources; ++resourceIdx)
	{
		Page::Resource::Type type = request.resources[resourceIdx];
		Page::Resource& resource = response.resources[resourceIdx];

		// Search for a resource provider registered for this type
		ResourceProvider* resourceProvider = resourceProviders[type];

		if (resourceProvider != NULL)
		{
			// Set the data pointer of the resource to the current position of the write stream
			// TODO: Check if the resource fits in the buffer
			resource.data = dataBuffer.writeStream.DataPointer();
			resource.length = resourceProvider->SerializeResource(type, dataBuffer.writeStream);

			// If the provider couldn't supply the resource, we handle it as an invalid type
			if (resource.length == 0)
				resource.type = Page::Resource::INVALID_RESOURCE;
			else
				resource.type = type;
		}
		else
		{
			resource.length = 0;
			resource.type = Page::Resource::INVALID_RESOURCE;
		}

	}

	// Send a response message with our response struct as payload
	SendResponseMessage(requestMessage, response);
}

void SerialInterface::ProcessCommandRequest(const Message& requestMessage)
{
	dataBuffer.Clear();

	Command::RequestMessage request(dataBuffer);
	Command::ResponseMessage response;

	// Deserialize the request struct from the payload
	MemoryStream stream(requestMessage.payload, requestMessage.payloadLength);
	bool result = request.Deserialize(stream);

	if (!result)
	{
		Debug::Print("Failed to deserialize command request!\n");
		return;
	}

	// Search for a command handler registered for this type
	CommandHandler* commandHandler = commandHandlers[request.type];

	if (commandHandler != NULL)
	{
		// Pass the request to the command handler
		if (commandHandler->HandleCommand(request))
			response.state = Command::COMMAND_OK;
		else
			response.state = Command::COMMAND_ERROR;
	}
	else
	{
		response.state = Command::COMMAND_UNKOWN;
	}

	// Send a response message with our response struct as payload
	SendResponseMessage(requestMessage, response);
}

void SerialInterface::ProcessLogRequest(const Message& requestMessage)
{

}

bool SerialInterface::ReadMessage(Message& message)
{
	if (!headersRead && !ReadMessageHeaders(message))
		return false;

	// Check if there is enough data in the buffer for the complete message
	if (messageBuffer.readStream.Available() < message.payloadLength)
		return false;

	// Read the message into the shared payload buffer
	messageBuffer.readStream.Read(payloadBuffer, message.payloadLength);
	message.payload = payloadBuffer;

	return true; 
}

bool SerialInterface::ReadMessageHeaders(Message& message)
{
	// We need to sync the read stream on a message boundary
	if (!SyncMessageStream())
		return false;

	// Attempt to read the message headers
	if (!message.Deserialize(messageBuffer.readStream))
		return false;

	// Discard header data from the buffer
	messageBuffer.Trim();

	// Verify CRC the message checksum
	Util::crc crc = CalculateMessageCRC(message);
	
	if (crc != message.crc && false)
	{
		// TODO: error handling?
		//assert(false);
		Debug::Print("Invalid CRC received!\n");
		return false;
	}

	headersRead = true;

	return true;
}

bool SerialInterface::SyncMessageStream()
{
	uint32_t bytesSkipped = 0;

	// Make sure the read buffer is synced with a message boundary
	while (messageBuffer.readStream.Available() >= sizeof(uint32_t))
	{
		// As long as we aren't at a message boundary we can discard the data before the current stream point
		messageBuffer.Trim();

		uint32_t magic = messageBuffer.readStream.ReadUInt32(true);

		// If the read number conforms to the magic number, we probably are at a message start
		if (magic == Message::MESSAGE_MAGIC)
		{
			if (bytesSkipped > 0)
				Debug::Print("Synced message stream by skipping %u bytes\n", bytesSkipped);

			return true;
		}

		// Check if there is more data available
		if (messageBuffer.readStream.Available() == 0)
			break;

		// Advance one byte and try again
		messageBuffer.readStream.ReadByte(); 
		++bytesSkipped;
	} 

	// Failed to sync, no message bounday present in the read buffer yet
	return false;
}

void SerialInterface::SendMessage(Message& message)
{
	message.id 		= GetNextMessageID();
	message.crc 	= CalculateMessageCRC(message);
	
	// Serialize the message headers to the serial port
	message.Serialize(serialPort);

	// Write the message payload to the serial port
	serialPort.Write(message.payload, message.payloadLength);
}

void SerialInterface::SendResponseMessage(const Message& requestMessage, Serializable& payload)
{
	// Create a message struct for the response header
	Message responseMessage;
	responseMessage.phase 			= Message::PHASE_RESPONSE;
	responseMessage.type 			= requestMessage.type;
	responseMessage.id 				= requestMessage.id;
	responseMessage.payloadLength 	= payload.SerializedSize();
	responseMessage.crc 			= CalculateMessageCRC(responseMessage);

	// Serialize the message headers to the serial port
	responseMessage.Serialize(serialPort);

	// Serialize the message payload to the serial port
	payload.Serialize(serialPort);
}

void SerialInterface::PurgeMessageBuffer()
{
	messageBuffer.Clear();
	headersRead = false;

	Debug::Print("Purging message buffer!\n");
}

Util::crc SerialInterface::CalculateMessageCRC(const Message& message) const
{
	Util::crc crc = 0;

	// Add all the message header fields to the crc
	crc = Util::UpdateCRC(crc, message.magic);
	crc = Util::UpdateCRC(crc, (uint8_t) message.phase);
	crc = Util::UpdateCRC(crc, (uint8_t) message.type);
	crc = Util::UpdateCRC(crc, message.id);
	crc = Util::UpdateCRC(crc, message.payloadLength);

	return crc;
}
