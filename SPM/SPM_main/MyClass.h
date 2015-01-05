/*
 * MyClass.h
 *
 * Created: 10/10/2014 9:10:14 AM
 * Author: Дмитрий Владимирович
 */ 

#ifndef _MYCLASS_h
#define _MYCLASS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class MyClass
{
 private:


 public:
	void setup();
	void loop();
};

extern MyClass myClass;

#endif

