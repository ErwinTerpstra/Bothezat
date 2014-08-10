#ifndef _MODULE_H_
#define _MODULE_H_

#include "base_module.h"
#include "config.h"

namespace bothezat
{
	
template <class Derived>
class Module : public BaseModule
{
public:

private:
	static Derived* instance;

protected:
	const Config& config;

	Module() : config(Config::Instance())
	{

	}

public:

	static Derived& Instance()
	{
		static Derived instance;

		return instance; 
	}
};

}

#endif