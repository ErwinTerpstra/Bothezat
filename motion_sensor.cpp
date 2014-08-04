#include "Arduino.h"
#include "bothezat.h"

#include "motion_sensor.h"

#include "i2c.h"

#include "mpu6050.h"

using namespace bothezat;

MotionSensor::MotionSensor() : orientation()
{
	
}

void MotionSensor::Setup()
{
	SetupMPU();
}

void MotionSensor::Loop(uint32_t dt)
{
	ReadMPU();

	float deltaSeconds = dt / 1000000.0f;
	float scale = gyroScale * deltaSeconds * DEG_2_RAD;

	// Convert axis rotations to quaternions
	Quaternion yaw 		= Quaternion::AngleAxis(yaw,	mpuData.gyroZ * scale, Vector3::Up());
	Quaternion pitch 	= Quaternion::AngleAxis(pitch,	mpuData.gyroX * scale, Vector3::Right());
	Quaternion roll 	= Quaternion::AngleAxis(roll,	mpuData.gyroY * scale, Vector3::Forward());

	// Apply relative rotations to saved orientation
	Quaternion rotation = yaw * pitch * roll;
	orientation = rotation * orientation;

	// Convert accelerometer values to G normalized
	acceleration.x = mpuData.accelX * accelScale;
	acceleration.y = mpuData.accelY * accelScale;
	acceleration.z = mpuData.accelZ * accelScale;

	// Only use accelerometer values if total acceleration is below threshold
	float magnitude = acceleration.Length();
	if (magnitude < Config::MS_ACCEL_MAX)
	{
		Vector3 forward = orientation * Vector3::Forward();
		Vector3 up = -acceleration;

		forward.Normalize();
		up.Normalize();

		Vector3 right = Vector3::Cross(forward, up);
		forward = Vector3::Cross(up, right);

		Quaternion accelOrientation = Quaternion::LookAt(forward, up);

		orientation = accelOrientation;
		//Quaternion::Slerp(orientation, orientation, accelOrientation, deltaSeconds);
	}
	else
		Debug::Print("Dropping accel values\n");
}

void MotionSensor::SetupMPU()
{
	// Default at MPU6050 settings at power-up:
	//    Gyro at 250 degrees second
	//    Acceleration at 2g
	//    Clock source at internal 8MHz
	//    The device is in sleep mode.
	//

	// Set the accelerometer range to 8G and the gyro range to 1000 degrees/s
	accelRange = 8;
	accelScale = (1.0f / INT16_MAX) * accelRange;

	gyroRange = 1000; 
	gyroScale = (1.0f / INT16_MAX) * gyroRange;
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_8G);
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_1000);

	// Clear the 'sleep' register of the MPU6050 to start recording data
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);
}

void MotionSensor::ReadMPU()
{
	uint8_t error = I2C::Read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, (uint8_t*) &mpuData, sizeof(mpuData));

	if (error != I2C::ERR_OK)
		Serial.println(error);

	// Correct endianness difference between uC and mpu data
	Util::SwapEndianness((uint8_t*) &mpuData.accelX, sizeof(mpuData.accelX));
	Util::SwapEndianness((uint8_t*) &mpuData.accelY, sizeof(mpuData.accelY));
	Util::SwapEndianness((uint8_t*) &mpuData.accelZ, sizeof(mpuData.accelZ));

	Util::SwapEndianness((uint8_t*) &mpuData.temperature, sizeof(mpuData.temperature));

	Util::SwapEndianness((uint8_t*) &mpuData.gyroX, sizeof(mpuData.gyroX));
	Util::SwapEndianness((uint8_t*) &mpuData.gyroY, sizeof(mpuData.gyroY));
	Util::SwapEndianness((uint8_t*) &mpuData.gyroZ, sizeof(mpuData.gyroZ));
}

void MotionSensor::PrintOrientation()
{
	float yaw, pitch, roll;
	orientation.ToEulerAngles(yaw, pitch, roll);

	Debug::Print("Orientation:\n");
	Debug::Print("Yaw = %.4f\n",		yaw * RAD_2_DEG);
	Debug::Print("Pitch = %.4f\n",		pitch * RAD_2_DEG);
	Debug::Print("Roll = %.4f\n",		roll * RAD_2_DEG);
	Debug::Print("\n");

	Debug::Print("Acceleration: %.4f;%.4f;%.4f;\n",	acceleration.x, acceleration.y, acceleration.z);
}