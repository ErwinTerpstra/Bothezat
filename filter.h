#ifndef _FILTER_H_
#define _FITLER_H_

#include "Arduino.h"

namespace bothezat
{
	
template<typename T>
class Filter
{

public: 
	enum Type
	{
		PASS_THROUGH,
		LOW_PASS,
		HIGH_PASS
	};

private:

	Type type;

	float rc;

	T previousInput, previousFiltered;

  	T (Filter<T>::* filter)(T, float);

public:

	Filter(Type type, float rc) : type(type), rc(rc)
	{
		switch (type)
		{
			case PASS_THROUGH:	filter = &Filter<T>::PassThroughFilter;		break;
			case LOW_PASS:		filter = &Filter<T>::LowPassFilter;			break;
			case HIGH_PASS:		filter = &Filter<T>::HighPassFilter;		break;
		}
	}

	T Sample(T input, float dt)
	{
		previousFiltered = (this->*filter)(input, Alpha(dt));
		previousInput = input;

		return previousFiltered;
	}

	void SetRC(float rc) { this->rc = rc; }

	__inline float Alpha(float dt) const { return rc / (rc + dt); }

	__inline T PassThroughFilter(T input, float alpha)
	{
		return input;
	}

	__inline T LowPassFilter(T input, float alpha)
	{
		return alpha * previousFiltered + (1 - alpha) * input;
	}

	__inline T HighPassFilter(T input, float alpha)
	{
		return alpha * previousFiltered + alpha * (input - previousInput);
	}

};

}

#endif