#ifndef _MODULE_H_
#define _MODULE_H_

namespace bothezat
{
	
template <class Derived>
class Module
{
public:

private:
	static Derived* instance;

protected:

	Module() { }

public:

	virtual void Setup() = 0;
	virtual void Loop(uint32_t dt) = 0;


	static Derived& Instance()
	{
		static Derived instance;

		return instance; 
	}
};

}

#endif