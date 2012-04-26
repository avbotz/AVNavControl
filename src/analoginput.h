#ifndef ANALOGINPUT_H
#define ANALOGINPUT_H

#include "mbed.h"

class AnalogInput {
public:
	AnalogInput(PinName pin, float thresh);
	~AnalogInput();
	float getValueRaw();
	bool getValueThresh();
private:
	float threshold;
	AnalogIn* p_analog;
};

extern AnalogInput kill;
extern AnalogInput pressure;

#endif
