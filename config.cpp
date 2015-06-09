#include "config.h"

#include "command.h"

using namespace bothezat;

Config Config::instance;

DueFlashStorage flash;

Config::Config() : buffer(NULL), SerializableResource(Page::Resource::CONFIG, *this)
{
	bufferSize = SerializedSize();

	buffer = static_cast<uint8_t*>(malloc(bufferSize));
	bufferStream = MemoryStream(buffer, bufferSize);
}

Config::~Config()
{
	if (buffer != NULL)
	{
		free(buffer);
		buffer = NULL;
	}
}

bool Config::ReadEEPROM()
{
	// Read buffer from flash
	for (uint32_t offset = 0; offset < bufferSize; ++offset)
		buffer[offset] = flash.read(offset);

	// Read config from buffer
	bufferStream.Seek(-bufferStream.Offset());
	return Deserialize(bufferStream);
}

void Config::WriteEEPROM()
{
	// Write config to buffer
	bufferStream.Seek(-bufferStream.Offset());
	Serialize(bufferStream);

	// Write buffer to flash
	flash.write(0, buffer, bufferSize);

	Debug::Print("Config written!\n");
}

void Config::LoadDefaults()
{
	/*
	 * Config management
	 */
	MAGIC 						= CONFIG_MAGIC;
	VERSION 					= LATEST_VERSION;

	/*
	 * System
	 */
	SYS_LOOP_TIME				= 0;			// Minimum loop time (us) the system is trying to match, zero means as fast as possible

	/*
	 * Serial interface
	 */
	SR_BAUD_RATE 				= 115200; 		// Baud rate for serial communication

	/*
	 * Radio receiver 
	 */
	

	/*
	 * Motion sensor
	 */
	MS_CALIBRATION_SAMPLES		= 10;			// Amount of samples for IMU
	MS_CALIBRATION_INTERVAL		= 100;			// Time between IMU calibration samples
	MS_GYRO_FILTER_RC			= 2.5f;			// RC for gyro high pass filter
	MS_ACCEL_CORRECTION_RC		= 0.0002f;		// Lower means slower correction to gyro by accelerometer
	MS_ACCEL_MAX				= 0.15f;		// Accelerometer values with a larger deviation from 1G than this will get discarded

	/*
	 * Flight system
	 */
	FS_MAN_ANGULAR_VELOCITY		= Vector3(50.0f, 50.0f, 50.0f);	// Angular velocity at max stick input for manual mode
	FS_ATTI_MAX_PITCH			= 45.0f;						// Pitch angle at max stick input for atti mode
	FS_ATTI_MAX_ROLL			= 45.0f;						// Roll angle at max stick input for atti mode

	/*
	 * Motor controller
	 */
	MC_PWM_FREQUENCY			= 50;
	MC_PWM_PERIOD				= 20000;
	MC_PWM_MIN_COMMAND			= 950;
	MC_PWM_MIN_OUTPUT			= 1100;
	MC_PWM_MAX_COMMAND			= 2050;

	MC_PID_CONFIGURATION[0] 	= PidConfiguration(1.0f, 0.005f, 1.0f);
	MC_PID_CONFIGURATION[1] 	= PidConfiguration(1.0f, 0.005f, 0.0f);
	MC_PID_CONFIGURATION[2] 	= PidConfiguration(1.0f, 0.005f, 1.0f);
}


void Config::Serialize(BinaryWriteStream& stream) const
{
	/*
	 * Config management
	 */
	stream.Write(MAGIC);
	stream.Write(VERSION);

	/*
	 * System
	 */
	stream.Write(SYS_LOOP_TIME);

	/*
	 * Serial interface
	 */
	stream.Write(SR_BAUD_RATE);

	/*
	 * Radio receiver 
	 */
	for (uint8_t channel = 0; channel < Constants::RX_MAX_CHANNELS; ++channel)
		RX_CHANNEL_CALIBRATION[channel].Serialize(stream);

	/*
	 * Motion sensor
	 */
	stream.Write(MS_CALIBRATION_SAMPLES);
	stream.Write(MS_CALIBRATION_INTERVAL);
	stream.Write(MS_GYRO_FILTER_RC);	
	stream.Write(MS_ACCEL_CORRECTION_RC);
	stream.Write(MS_ACCEL_MAX);

	/*
	 * Flight system
	 */
	FS_MAN_ANGULAR_VELOCITY.Serialize(stream);

	stream.Write(FS_ATTI_MAX_PITCH);
	stream.Write(FS_ATTI_MAX_ROLL);

	/*
	 * Motor controller
	 */
	stream.Write(MC_PWM_FREQUENCY);
	stream.Write(MC_PWM_PERIOD);
	stream.Write(MC_PWM_MIN_COMMAND);
	stream.Write(MC_PWM_MIN_OUTPUT);
	stream.Write(MC_PWM_MAX_COMMAND);

	for (uint8_t axis = 0; axis < 3; ++axis)
		MC_PID_CONFIGURATION[axis].Serialize(stream);
}

bool Config::Deserialize(BinaryReadStream& stream)
{
	
	/*
	 * Config management
	 */
	MAGIC = stream.ReadUInt32();

	if (MAGIC != CONFIG_MAGIC)
		return false;

	VERSION = stream.ReadUInt16();

	if (VERSION != LATEST_VERSION)
		return false;

	/*
	 * System
	 */
	SYS_LOOP_TIME 				= stream.ReadUInt16();

	/*
	 * Serial interface
	 */
	SR_BAUD_RATE 				= stream.ReadUInt32();

	for (uint8_t channel = 0; channel < Constants::RX_MAX_CHANNELS; ++channel)
		RX_CHANNEL_CALIBRATION[channel].Deserialize(stream);

	/*
	 * Motion sensor
	 */
	MS_CALIBRATION_SAMPLES 		= stream.ReadByte();
	MS_CALIBRATION_INTERVAL 	= stream.ReadUInt16();
	MS_GYRO_FILTER_RC 			= stream.ReadFloat();
	MS_ACCEL_CORRECTION_RC 		= stream.ReadFloat();
	MS_ACCEL_MAX 				= stream.ReadFloat();

	/*
	 * Flight system
	 */

	FS_MAN_ANGULAR_VELOCITY.Deserialize(stream);

	FS_ATTI_MAX_PITCH 			= stream.ReadFloat();
	FS_ATTI_MAX_ROLL			= stream.ReadFloat();

	/*
	 * Motor controller
	 */
	MC_PWM_FREQUENCY 			= stream.ReadUInt32();
	MC_PWM_PERIOD 				= stream.ReadUInt16();
	MC_PWM_MIN_COMMAND 			= stream.ReadUInt16();
	MC_PWM_MIN_OUTPUT 			= stream.ReadUInt16();
	MC_PWM_MAX_COMMAND 			= stream.ReadUInt16();

	for (uint8_t axis = 0; axis < 3; ++axis)
		MC_PID_CONFIGURATION[axis].Deserialize(stream);

	return true;
}

uint32_t Config::SerializedSize() const
{
	return 
		sizeof(uint32_t) + // MAGIC;

		sizeof(uint16_t) + // VERSION;

		/*
		 * System
		 */
		sizeof(uint16_t) + // SYS_LOOP_TIME;

		/*
		 * Serial interface
		 */
		sizeof(uint32_t) + // SR_BAUD_RATE;

		ChannelCalibration::Size() * Constants::RX_MAX_CHANNELS + // RX_CHANNEL_CALIBRATION[Constants.RX_MAX_CHANNELS];

		/*
		 * Motion sensor
		 */
		sizeof(uint8_t) + // MS_CALIBRATION_SAMPLES;

		sizeof(uint16_t) + // MS_CALIBRATION_INTERVAL;

		sizeof(float) + // MS_GYRO_FILTER_RC;
		
		sizeof(float) + // MS_ACCEL_CORRECTION_RC;

		sizeof(float) + // MS_ACCEL_MAX;

		/*
		 * Flight system
		 */

		Vector3::Size() + // FS_MAN_ANGULAR_VELOCITY;

		sizeof(float) + // FS_ATTI_MAX_PITCH;

		sizeof(float) + // FS_ATTI_MAX_ROLL;

		/*
		 * Motor controller
		 */
		sizeof(uint32_t) + // MC_PWM_FREQUENCY;

		sizeof(uint16_t) + // MC_PWM_PERIOD;

		sizeof(uint16_t) + // MC_PWM_MIN_COMMAND;

		sizeof(uint16_t) + // MC_PWM_MIN_OUTPUT;

		sizeof(uint16_t) + // MC_PWM_MAX_COMMAND;

		PidConfiguration::Size() * 3 + // MC_PID_CONFIGURATION[3];
	0;
}

bool Config::HandleCommand(Command::RequestMessage& command)
{
	if (command.type != Command::SAVE_CONFIG)
		return false;

	if (!Deserialize(command.buffer.readStream))
		return false;

	WriteEEPROM();

	return true;
}