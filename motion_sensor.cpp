#include "Arduino.h"
#include "bothezat.h"

#include "motion_sensor.h"

#include "i2c.h"

#include "mpu6050.h"

using namespace bothezat;

MotionSensor::MotionSensor() : 
	gyroFilter(Filter<Vector3>::PASS_THROUGH, Config::MS_GYRO_FILTER_RC), 
	orientation(), accelOrientation(), acceleration(), angularVelocity(),
	yaw(0.0f), pitch(0.0f), roll(0.0f)
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

	float deltaSeconds = dt / 1000000.0f;
	float scale = gyroScale * DEG_2_RAD;

	ReadGyro(angularVelocity);
	ReadAcceleration(acceleration);

	// Filter the gyro data
	angularVelocity = gyroFilter.Sample(angularVelocity, deltaSeconds);

	// Convert axis rotations to quaternions
	Quaternion yaw 		= Quaternion::AngleAxis(yaw,	angularVelocity.y * deltaSeconds, Vector3::Up());
	Quaternion pitch 	= Quaternion::AngleAxis(pitch,	angularVelocity.x * deltaSeconds, Vector3::Right());
	Quaternion roll 	= Quaternion::AngleAxis(roll,	angularVelocity.z * deltaSeconds, Vector3::Forward());

	// Apply relative rotations to saved orientation
	Quaternion rotation = roll * pitch * yaw;
	orientation = rotation * orientation;

	// Only use accelerometer values if total acceleration is below threshold
	float magnitude = acceleration.Length();
	if (fabs(1.0f - magnitude) < Config::MS_ACCEL_MAX)
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

			Quaternion::Slerp(orientation, orientation, accelOrientation, Config::MS_ACCEL_CORRECTION_RC / (Config::MS_ACCEL_CORRECTION_RC + deltaSeconds));
		}
	}
}

void MotionSensor::Calibrate()
{
	Debug::Print("Calibrating accelerometer with %d samples\n", Config::MS_CALIBRATION_SAMPLES);

	float scale = 0.0f;

	// TODO: calibrate initial orientation from accelerometer

	// Read multiple samples to find real accelerometer scale
	for (uint8_t sampleIdx = 0; sampleIdx < Config::MS_CALIBRATION_SAMPLES; ++sampleIdx)
	{
		// Read new values from i2c
		ReadMPU();

		// Read the length of the acceleration vector
		ReadAcceleration(acceleration);
		scale += acceleration.Length() / Config::MS_CALIBRATION_SAMPLES;

		// Wait a while until next sample
		delay(Config::MS_CALIBRATION_INTERVAL);
	}

	Debug::Print("Accelerometer calibrated at %.4f\n", scale);

	// Apply calibration scale to the conversion scale
	accelScale *= 1.0f / scale;
}

void MotionSensor::SetupMPU()
{
	// Default at MPU6050 settings at power-up:
	//    Gyro at 250 degrees second
	//    Acceleration at 2g
	//    Clock source at internal 8MHz
	//    The device is in sleep mode.
	//

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

	// Convert data to right handed system
	//mpuData.accelZ = -mpuData.accelZ;
}

void MotionSensor::PrintOrientation()
{
	orientation.ToEulerAngles(yaw, pitch, roll);
	Debug::Print("Orientation:\n");
	Debug::Print("Yaw = %.4f; Pitch = %.4f; Roll = %.4f\n", yaw * RAD_2_DEG, pitch * RAD_2_DEG, roll * RAD_2_DEG);

	accelOrientation.ToEulerAngles(yaw, pitch, roll);
	Debug::Print("Accel. orientation:\n");
	Debug::Print("Yaw = %.4f; Pitch = %.4f; Roll = %.4f\n", yaw * RAD_2_DEG, pitch * RAD_2_DEG, roll * RAD_2_DEG);
	Debug::Print("\n");

	Debug::Print("Acceleration: %.4f;%.4f;%.4f;\tLength:%.4f\n", acceleration.x, acceleration.y, acceleration.z, acceleration.Length());
	Debug::Print("Ang. vel.: %.4f;%.4f;%.4f;\tLength:%.4f\n", angularVelocity.x, angularVelocity.y, angularVelocity.z, angularVelocity.Length());
}