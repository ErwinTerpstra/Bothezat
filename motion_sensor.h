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

	uint8_t accelRange;
	uint16_t gyroRange;

	Quaternion orientation;

protected:
	MotionSensor();

public:

	virtual void Setup();
	virtual void Loop();

private:
	void SetupMPU();
	void ReadMPU();
};


}

#endif