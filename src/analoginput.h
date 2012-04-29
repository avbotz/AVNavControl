#ifndef ANALOGINPUT_H
#define ANALOGINPUT_H

#include "mbed.h"

class AnalogInput {
public:
	AnalogInput(PinName pin, float thresh);
	~AnalogInput();
	float getValueRaw();
	bool getValueThresh();
protected:
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

class AnalogKillSwitch : public AnalogInput {
public:
	AnalogKillSwitch(PinName Vin, PinName Vout, float thresh);
	~AnalogKillSwitch();
	
	// These functions do averaging.
	float getValueRaw();
	bool getValueThresh();
	
private:
	AnalogOut* p_Vout;
	float value;
};

extern AnalogKillSwitch kill;
extern AnalogPressureSensor pressure;

#endif
