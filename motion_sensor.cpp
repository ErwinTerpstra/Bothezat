#include "Arduino.h"
#include "bothezat.h"

#include "motion_sensor.h"

#include "i2c.h"

#include "mpu6050.h"

using namespace bothezat;

MotionSensor::MotionSensor()
{
	
}

void MotionSensor::Setup()
{
	SetupMPU();
}

void MotionSensor::Loop()
{
	ReadMPU();
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
	gyroRange = 1000;
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_8G);
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_1000);

	// Clear the 'sleep' register of the MPU6050 to start recording data
	I2C::WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);
}

void MotionSensor::ReadMPU()
{
	uint8_t error = I2C::Read(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, (uint8_t*) &mpuData, sizeof(mpuData));

	// Correct endianness difference between uC and mpu data
	Util::SwapEndianness((uint8_t*) &mpuData.accelX, sizeof(mpuData.accelX));
	Util::SwapEndianness((uint8_t*) &mpuData.accelY, sizeof(mpuData.accelY));
	Util::SwapEndianness((uint8_t*) &mpuData.accelZ, sizeof(mpuData.accelZ));

	Util::SwapEndianness((uint8_t*) &mpuData.temperature, sizeof(mpuData.temperature));

	Util::SwapEndianness((uint8_t*) &mpuData.gyroX, sizeof(mpuData.gyroX));
	Util::SwapEndianness((uint8_t*) &mpuData.gyroY, sizeof(mpuData.gyroY));
	Util::SwapEndianness((uint8_t*) &mpuData.gyroZ, sizeof(mpuData.gyroZ));

	float INT16_MAXF = (float) INT16_MAX;

	// Print the raw acceleration values
	Serial.print(F("accel x,y,z: "));
	Serial.print(mpuData.accelX / INT16_MAXF, 2);
	Serial.print(F(", "));
	Serial.print(mpuData.accelY / INT16_MAXF, 2);
	Serial.print(F(", "));
	Serial.print(mpuData.accelZ / INT16_MAXF, 2);
	Serial.println(F(""));

}