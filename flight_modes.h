#ifndef _FLIGHT_MODES_H_
#define _FLIGHT_MODES_H_

#include "bothezat.h"

#include "receiver.h"

namespace bothezat
{


// Base class for flight modes
class FlightMode
{
public:

	enum ID
	{
		MANUAL = 0,
		ANGLE,

		LAST_FLIGHT_MODE
	};

private:
	ID id;

	Quaternion desiredOrientation;
	Rotation desiredRotation;

protected:
	Receiver& receiver;

	Config& config;

protected:
	FlightMode(ID id, Receiver& receiver) : 
		id(id), desiredOrientation(),
		receiver(receiver), config(Config::Instance())
	{

	}

	void SetDesiredOrientation(const Quaternion& desiredOrientation) 
	{
		this->desiredOrientation = desiredOrientation; 
		desiredOrientation.ToEulerAngles(desiredRotation);
	}

public:
	virtual void Setup() = 0;
	virtual void Loop(uint32_t dt) = 0;

	virtual void OnEnter() { };
	virtual void OnExit() { };

	virtual const char* Name() = 0;

	const Quaternion& DesiredOrientation() const { return desiredOrientation; }
	const Rotation& DesiredRotation() const { return desiredRotation; }
};

// Sticks control angular velocity
class ManualMode : public FlightMode
{

private:
	Quaternion setOrientation;

public:
	ManualMode(Receiver& receiver) : FlightMode(MANUAL, receiver)
	{

	}

	virtual void Setup()
	{

	}

	virtual void Loop(uint32_t dt)
	{
		// Get rotation stick inputs 
		Rotation rotation;
		rotation.yaw = receiver.NormalizedChannel(Receiver::RUDDER);
		rotation.pitch = receiver.NormalizedChannel(Receiver::ELEVATOR);
		rotation.roll = receiver.NormalizedChannel(Receiver::AILERON);

		float deltaSeconds = dt * 1e-6f;

		// Scale velocity by max velocity and delta time
		rotation = rotation.ComponentMultiply(config.FS_MAN_ANGULAR_VELOCITY);
		rotation *= deltaSeconds;

		// Create a combined rotation and apply it to our previous set orientation
		Quaternion rotationQ = Quaternion::FromEulerAngles(rotationQ, rotation);
		setOrientation = rotationQ * setOrientation;

		SetDesiredOrientation(setOrientation);
	}

	virtual const char* Name() { return "Manual"; }

};

// Sticks control angle directly
class AngleMode : public FlightMode
{

public:
	AngleMode(Receiver& receiver) : FlightMode(ANGLE, receiver)
	{

	}

	virtual void Setup()
	{

	}

	virtual void Loop(uint32_t dt)
	{ 
		Rotation rotation = DesiredRotation(); // Copy previous rotation for yaw
		rotation.pitch = receiver.NormalizedChannel(Receiver::ELEVATOR) * config.FS_ATTI_MAX_PITCH;
		rotation.roll = receiver.NormalizedChannel(Receiver::AILERON) * config.FS_ATTI_MAX_ROLL;

		// Apply relative yaw as we would in manual mode
		float deltaSeconds = dt * 1e-6f;
		float yaw = receiver.NormalizedChannel(Receiver::RUDDER);
		rotation.yaw += yaw * config.FS_MAN_ANGULAR_VELOCITY.yaw * deltaSeconds;

		// Create a combined orientation which represents the orientation of the sticks
		Quaternion orientation = Quaternion::FromEulerAngles(orientation, rotation);
		SetDesiredOrientation(orientation);
	}

	virtual const char* Name() { return "Angle"; }

};


}

#endif