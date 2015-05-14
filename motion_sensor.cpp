#include "Arduino.h"
#include "bothezat.h"

#include "motion_sensor.h"

#include "i2c.h"

#include "mpu6050.h"

using namespace bothezat;

MotionSensor::MotionSensor() : 
	orientation(), accelOrientation(), acceleration(), angularVelocity(),
	gyroOffset(), gyroRange(0), accelRange(0), gyroScale(1.0f), accelScale(1.0f),
	angularVelocityFilter(Filter<Vector3>::HIGH_PASS, 0.1f), accelerationFilter(Filter<Vector3>::LOW_PASS, 0.1f)
{
	
}

void MotionSensor::Setup()
{
	SetupMPU();
	Calibrate();
}

void MotionSensor::Loop(uint32_t dt)
{
	// Read new values from i2c
	ReadMPU();

	float deltaSeconds = dt * 1e-6f;
	float scale = gyroScale * DEG_2_RAD;

	// Convert raw data to scaled and calibrated data
	ReadGyro(angularVelocity);
	ReadAcceleration(acceleration);

	angularVelocity = angularVelocityFilter.Sample(angularVelocity, deltaSeconds);
	acceleration = accelerationFilter.Sample(acceleration, deltaSeconds);

	// Convert axis rotations to quaternion
	Quaternion rotation = Quaternion::FromEulerAngles(rotation,
													  angularVelocity.y * deltaSeconds, 
													  angularVelocity.x * deltaSeconds, 
												  	  angularVelocity.z * deltaSeconds);

	// Apply relative rotations to saved orientation
	orientation = rotation * orientation;

	// Only use accelerometer values if total acceleration is below threshold
	float magnitude = acceleration.Length();
	if (fabs(1.0f - magnitude) < config.MS_ACCEL_MAX)
	{
		Vector3 forward = orientation * Vector3::Forward();
		Vector3 up = -acceleration;

		// Normalize axis vectors
		forward.Normalize();
		up.Normalize();

		// Check if the forward and up vector are paralel
		if (fabs(Vector3::Dot(forward, up)) < 1.0f - FLT_EPSILON)
		{
			// Orthonormalize forward vector to up vector by calculating their shared right vector
			Vector3 right = Vector3::Cross(forward, up);
			forward = Vector3::Cross(up, right);

			Quaternion::RotationBetween(accelOrientation, up, Vector3::Up());

			Quaternion::Slerp(orientation, orientation, accelOrientation, config.MS_ACCEL_CORRECTION_RC / (config.MS_ACCEL_CORRECTION_RC + deltaSeconds));
		}
	}
}

uint16_t MotionSensor::SerializeResource(Page::Resource::Type type, BinaryWriteStream& stream)
{
	switch (type)
	{
		case Page::Resource::ORIENTATION:
			orientation.Serialize(stream);

			return orientation.SerializedSize();

		case Page::Resource::ACCEL_ORIENTATION:
			accelOrientation.Serialize(stream);

			return accelOrientation.SerializedSize();

		case Page::Resource::ACCELERATION:
			acceleration.Serialize(stream);

			return acceleration.SerializedSize();

		case Page::Resource::ANGULAR_VELOCITY:
			acceleration.Serialize(stream);

			return acceleration.SerializedSize();
	} 

	return 0;
}

void MotionSensor::Debug() const
{
	float yaw, pitch, roll;
	orientation.ToEulerAngles(yaw, pitch, roll);
	Debug::Print("Orientation:\n");
	Debug::Print("Yaw = %.4f; Pitch = %.4f; Roll = %.4f\n", yaw, pitch, roll);

	accelOrientation.ToEulerAngles(yaw, pitch, roll);
	Debug::Print("Accel. orientation:\n");
	Debug::Print("Yaw = %.4f; Pitch = %.4f; Roll = %.4f\n", yaw, pitch, roll);
	Debug::Print("\n");

	Debug::Print("Acceleration: %.4f;%.4f;%.4f;\tLength:%.4f\n", acceleration.x, acceleration.y, acceleration.z, acceleration.Length());
	Debug::Print("Ang. vel.: %.4f;%.4f;%.4f;\tLength:%.4f\n", angularVelocity.x, angularVelocity.y, angularVelocity.z, angularVelocity.Length());
}

void MotionSensor::Calibrate()
{
	Debug::Print("Calibrating accelerometer with %d samples\n", config.MS_CALIBRATION_SAMPLES);

	float scale = 0.0f;
	Vector3 averageGyro = Vector3::Zero();

	// TODO: calibrate initial orientation from accelerometer

	// Read multiple samples to find real accelerometer scale
	for (uint8_t sampleIdx = 0; sampleIdx < config.MS_CALIBRATION_SAMPLES; ++sampleIdx)
	{
		// Read new values from i2c
		ReadMPU();

		ReadAcceleration(acceleration);
		ReadGyro(angularVelocity);

		// The current length of the acceleration vector is the error in scale
		scale += acceleration.Length() / config.MS_CALIBRATION_SAMPLES;

		// Angular velocity in rest is the offset the gyro has
		averageGyro += angularVelocity * (1.0f / config.MS_CALIBRATION_SAMPLES);

		// Wait a while until next sample
		delay(config.MS_CALIBRATION_INTERVAL);
	}

	Debug::Print("Accelerometer calibrated at %.4f\n", scale);
	Debug::Print("Gyroscope offset calibrated at: %.4f;%.4f;%.4f\n", averageGyro.x, averageGyro.y, averageGyro.z);

	// Apply calibration scale to the conversion scale
	accelScale *= 1.0f / scale;
	gyroOffset = averageGyro;
}

void MotionSensor::SetupMPU()
{
	// Default at MPU6050 settings at power-up:
	//    Gyro at 250 degrees second
	//    Acceleration at 2g
	//    Clock source at internal 8MHz
	//    The device is in sleep mode.
	//

	// TODO: generalize this to the config or a constant
	// Set the accelerometer range to 4G and the gyro range to 500 degrees/s
	accelRange = 4;
	accelScale = (1.0f / INT16_MAX) * accelRange;

	gyroRange = 500; 
	gyroScale = (1.0f / INT16_MAX) * gyroRange;
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_4G);
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_500);

	// Clear the 'sleep' register of the MPU6050 to start recording data
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);
}

void MotionSensor::ReadMPU()
{
	uint8_t error = I2C::Read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, (uint8_t*) &mpuData, sizeof(mpuData));

	if (error != I2C::ERR_OK)
		Debug::Print("I2C error while reading MPU: %d\n", error);

	// Correct endianness difference between uC and mpu data
	Util::SwapEndianness((uint8_t*) &mpuData.accelX, sizeof(mpuData.accelX));
	Util::SwapEndianness((uint8_t*) &mpuData.accelY, sizeof(mpuData.accelY));
	Util::SwapEndianness((uint8_t*) &mpuData.accelZ, sizeof(mpuData.accelZ));

	Util::SwapEndianness((uint8_t*) &mpuData.temperature, sizeof(mpuData.temperature));

	Util::SwapEndianness((uint8_t*) &mpuData.gyroX, sizeof(mpuData.gyroX));
	Util::SwapEndianness((uint8_t*) &mpuData.gyroY, sizeof(mpuData.gyroY));
	Util::SwapEndianness((uint8_t*) &mpuData.gyroZ, sizeof(mpuData.gyroZ));
}