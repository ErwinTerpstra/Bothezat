#ifndef _MOTION_SENSOR_H_
#define _MOTION_SENSOR_H_

#include "Arduino.h"

#include "module.h"

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

	Quaternion orientation;
	Vector3 acceleration;

protected:
	MotionSensor();

public:

	virtual void Setup();
	virtual void Loop(uint32_t dt);

	void PrintOrientation();
	
private:
	void SetupMPU();
	void ReadMPU();
};


}

#endif