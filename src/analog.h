#ifndef ANALOGINPUT_H
#define ANALOGINPUT_H

#include "mbed.h"

/*
 * Generic class for devices that output analog to the mbed.
 * Currently, these are the pressure sensor and the killswitch.
 */
class AnalogInput {
public:
	AnalogInput(PinName pin);
	~AnalogInput();
	float getValueRaw();
protected:
	AnalogIn* p_analog;
};

/* 
 * The pressure sensor outputs voltage linearly with depth.
 * The variables must be manually calibrated.
 */
class AnalogPressureSensor : public AnalogInput {
public:
	// Pass in calibration data in the form of y = mx + b equation.
	AnalogPressureSensor(PinName pin, float calibration_m, float calibration_b);
	~AnalogPressureSensor();
	
	int getValueCalibrated();
private:
	float m, b;
};

/*
 * The kill switch is either killed or unkilled, determined by whether
 * the voltage is greater than some specified threshold.
 */
class AnalogKillSwitch : public AnalogInput {
public:
	AnalogKillSwitch(PinName Vin, PinName Vout, float thresh);
	~AnalogKillSwitch();
	
	// These functions do averaging.
	float getValueRaw();
	bool getValueThresh();
	
private:
	AnalogOut* p_Vout;
	float value, threshold;
};


void updateKill();

//The two analog devices are the killswitch and pressure sensor.
extern AnalogKillSwitch kill;
extern AnalogPressureSensor pressure;
extern volatile bool isAlive;

#endif
