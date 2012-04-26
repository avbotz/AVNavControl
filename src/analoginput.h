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

class AnalogPressureSensor : public AnalogInput {
public:
	// Pass in calibration data in the form of y = mx + b equation.
	AnalogPressureSensor(PinName pin, float calibration_m, float calibration_b);
	~AnalogPressureSensor();
	
	int getValueCalibrated();
private:
	float m, b;
};

extern AnalogInput kill;
extern AnalogPressureSensor pressure;

#endif
