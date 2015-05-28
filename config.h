#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Arduino.h"
#include "vector3.h"

#include "binary_stream.h"
#include "memory_stream.h"
#include "page.h"

#include <DueFlashStorage.h>

namespace bothezat
{

class Vector3;

class Config : public Serializable, public Deserializable, public SerializableResource
{	
	
public:
	struct Pins
	{
		enum Pin
		{
			LED_CONTROLLER	= 23,

			I2C_SDA			= 20,
			I2C_SCL			= 21,

			RX_PWM			= 48
		};
	};

	struct Constants
	{
		/*
		 *	Radio receiver
		 */
		static const uint8_t RX_PWM_AMOUNT = 2;

		static const uint8_t RX_MAX_CHANNELS = 8;

		/*
		 * Motor controller
		 */
		static const uint8_t MC_MOTOR_AMOUNT = 4;

	};

	struct ChannelCalibration : public Serializable, public Deserializable
	{
		uint16_t min, max, mid, deadband;

		ChannelCalibration()
		{
			min = 1050;
			mid = 1500;
			max = 1950;
			deadband = 30;
		}

		virtual void Serialize(BinaryWriteStream& stream) const
		{
			stream.Write(min);
			stream.Write(max);
			stream.Write(mid);
			stream.Write(deadband);
		}

		virtual bool Deserialize(BinaryReadStream& stream)
		{
			min = stream.ReadUInt16();
			max = stream.ReadUInt16();
			mid = stream.ReadUInt16();
			deadband = stream.ReadUInt16();

			return true;
		}

		__inline virtual uint32_t SerializedSize() const
		{
			return ChannelCalibration::Size();
		}

		static uint32_t Size()
		{
			return sizeof(uint16_t) * 4;
		}
	};

	struct PidConfiguration : public Serializable, public Deserializable
	{
		float kp, ki, kd;
		
		PidConfiguration() : kp(1.0f), ki(1.0f), kd(1.0f)
		{

		}

		PidConfiguration(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd)
		{

		}

		virtual void Serialize(BinaryWriteStream& stream) const
		{
			stream.Write(kp);
			stream.Write(ki);
			stream.Write(kd);
		}

		virtual bool Deserialize(BinaryReadStream& stream)
		{
			kp = stream.ReadFloat();
			ki = stream.ReadFloat();
			kd = stream.ReadFloat();

			return true;
		}

		__inline virtual uint32_t SerializedSize() const
		{
			return PidConfiguration::Size();
		}

		static uint32_t Size()
		{
			return sizeof(float) * 3;
		}

	};

	static const uint32_t CONFIG_MAGIC = 0xDEADBEEF;

	static const uint16_t LATEST_VERSION = 0x01;

	/*
	 * Config management
	 */
	uint32_t MAGIC;

	uint16_t VERSION;

	/*
	 * System
	 */
	uint16_t SYS_LOOP_TIME;

	/*
	 * Serial interface
	 */
	uint32_t SR_BAUD_RATE;

	ChannelCalibration RX_CHANNEL_CALIBRATION[Constants::RX_MAX_CHANNELS];

	/*
	 * Motion sensor
	 */
	uint8_t MS_CALIBRATION_SAMPLES;

	uint16_t MS_CALIBRATION_INTERVAL;

	float MS_GYRO_FILTER_RC;
	
	float MS_ACCEL_CORRECTION_RC;

	float MS_ACCEL_MAX;

	/*
	 * Flight system
	 */

	Vector3 FS_MAN_ANGULAR_VELOCITY;

	float FS_ATTI_MAX_PITCH;

	float FS_ATTI_MAX_ROLL;

	/*
	 * Motor controller
	 */
	uint32_t MC_PWM_FREQUENCY;

	uint16_t MC_PWM_PERIOD;

	uint16_t MC_PWM_MIN_COMMAND;

	uint16_t MC_PWM_MIN_OUTPUT;

	uint16_t MC_PWM_MAX_COMMAND;

	PidConfiguration MC_PID_CONFIGURATION[3];

private:
	uint8_t* buffer;

	uint32_t bufferSize;

	MemoryStream bufferStream;

private:
	Config();

	~Config();

	static Config instance;

public:
	bool ReadEEPROM();

	void WriteEEPROM();

	void LoadDefaults();

	virtual void Serialize(BinaryWriteStream& stream) const;

	virtual bool Deserialize(BinaryReadStream& stream);

	virtual uint32_t SerializedSize() const;

public:

	static Config& Instance()
	{
		return instance;
	}


};


}


#endif