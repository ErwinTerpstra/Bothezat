#ifndef _PWM_RECEIVER_H_
#define _PWM_RECEIVER_H_

#include "Arduino.h"
#include "receiver.h"

namespace bothezat
{
	
/*
 *	Class which reads PWM output from radio receivers
 */
class PwmReceiver : public Receiver<PwmReceiver>
{
friend class Module<PwmReceiver>;


public:


protected:
	PwmReceiver();

public:
	virtual void Setup();

	virtual void Loop();
};

}

#endif