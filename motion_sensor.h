#ifndef _MOTION_SENSOR_H_
#define _MOTION_SENSOR_H_

#include "Arduino.h"

#include "module.h"
#include "filter.h"

#include "mpu6050.h"

namespace bothezat
{
	
class MotionSensor : public Module<MotionSensor>
{
friend class Module<MotionSensor>;

private:
	MPU6050Data mpuData;

	uint16_t accelRange, gyroRange;
	float accelScale, gyroScale;

	Quaternion orientation, accelOrientation;
	Vector3 angularVelocity, acceleration;

	Filter<Vector3> gyroFilter;
	float yaw, pitch, roll;

protected:
	MotionSensor();

public:

	virtual void Setup();
	virtual void Loop(uint32_t dt);

	void Calibrate();
	void PrintOrientation();
	
private:
	void SetupMPU();
	void ReadMPU();

	__inline void ConvertVector(Vector3& v)
	{
		float tmp = v.y;

		v.y = -v.z;
		v.z = -tmp;
	}

	void ReadAcceleration(Vector3& acceleration)
	{
		// Convert accelerometer values to G normalized
		acceleration.x = mpuData.accelX;
		acceleration.y = mpuData.accelY;
		acceleration.z = mpuData.accelZ;

		acceleration *= accelScale;

		ConvertVector(acceleration);
	}

	void ReadGyro(Vector3& angularVelocity)
	{
		angularVelocity.x = mpuData.gyroX;
		angularVelocity.y = mpuData.gyroY;
		angularVelocity.z = mpuData.gyroZ;

		angularVelocity *= gyroScale * DEG_2_RAD;

		ConvertVector(angularVelocity);
	}
};


}

#endif