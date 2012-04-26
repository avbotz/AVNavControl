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

AnalogInput kill(p16, 0.5);
AnalogInput pressure(p17, 0.5);

#endif
