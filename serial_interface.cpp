#include "Arduino.h"
#include "bothezat.h"

#include "serial_interface.h"
#include "memory_stream.h"

#include "motion_sensor.h"

using namespace bothezat;

SerialInterface::SerialInterface() : payloadBuffer(NULL), serialPort(SerialPort::Instance())
{
	
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
	resourceBuffer.Allocate(RESOURCE_BUFFER_SIZE);

	payloadBuffer = static_cast<uint8_t*>(malloc(Message::MAX_PAYLOAD_LENGTH));

	serialPort.Begin(config.SR_BAUD_RATE);
}

void SerialInterface::Loop(uint32_t dt)
{
	ReadMessages();
	ProcessMessages();
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
	Page::RequestMessage request;
	Page::ResponseMessage response;

	// Deserialize the request struct from the payload
	MemoryStream stream(requestMessage.payload, requestMessage.payloadLength);
	request.Deserialize(stream);

	// We always respond with the same number of resources, even if some of them are invalid
	response.numResources = request.numResources;

	// Iterate through each resource in the request 
	for (uint32_t resourceIdx = 0; resourceIdx < request.numResources; ++resourceIdx)
	{
		Page::Resource::Type type = request.resources[resourceIdx];
		Page::Resource& resource = response.resources[resourceIdx];

		// TODO: Move this to a generic resource retrieve pattern
		switch (type)
		{
			case Page::Resource::ORIENTATION:
			{
				Quaternion orientation = MotionSensor::Instance().CurrentOrientation();

				resource.type = type;
				resource.length = orientation.SerializedSize();

				resource.data = resourceBuffer.writeStream.DataPointer();
				orientation.Serialize(resourceBuffer.writeStream);
				break;
			}

			case Page::Resource::ACCEL_ORIENTATION:
			{
				Quaternion accelOrientation = MotionSensor::Instance().AccelerometerOrientation();

				resource.type = type;
				resource.length = accelOrientation.SerializedSize();

				resource.data = resourceBuffer.writeStream.DataPointer();
				accelOrientation.Serialize(resourceBuffer.writeStream);
				break;
			}

			default:
				resource.type = Page::Resource::INVALID_RESOURCE;
				resource.length = 0;
				break;
		}
	}

	// Send a response message with our response struct as payload
	SendResponseMessage(requestMessage, response);
}

void SerialInterface::ProcessCommandRequest(const Message& requestMessage)
{

}

void SerialInterface::ProcessLogRequest(const Message& requestMessage)
{

}

bool SerialInterface::ReadMessage(Message& message)
{
	if (!message.headersRead && !ReadMessageHeaders(message))
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
	
	if (crc != message.crc)
	{
		// TODO: error handling?
		//assert(false);
		SendLog("Invalid CRC received!");

		return false;
	}
	
	message.headersRead = true;

	return true;
}

bool SerialInterface::SyncMessageStream()
{
	// Make sure the read buffer is synced with a message boundary
	while (messageBuffer.readStream.Available() <= sizeof(uint32_t));
	{
		// As long as we aren't at a message boundary we can discard the data before the current stream point
		messageBuffer.Trim();

		uint32_t magic = messageBuffer.readStream.ReadUInt32();

		// If the read number conforms to the magic number, we probably are at a message start
		if (magic == Message::MESSAGE_MAGIC)
			return true;

		// Advance one byte and try again
		messageBuffer.readStream.ReadByte(); 
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
	requestMessage.Serialize(serialPort);

	// Serialize the message payload to the serial port
	payload.Serialize(serialPort);
}

void SerialInterface::PurgeMessageBuffer()
{
	messageBuffer.Clear();
	lastReceivedMessage.headersRead = false;
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
