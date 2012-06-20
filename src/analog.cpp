#include "analog.h"

AnalogInput::AnalogInput(PinName pin) {
	p_analog = new AnalogIn(pin);
}

AnalogInput::~AnalogInput() {
	if (p_analog != NULL) {
		delete p_analog;
	}
}

/*
 * Returns the raw value of the device. Probably noisy.
 */
float AnalogInput::getValueRaw() {
	return p_analog->read();
}

AnalogPressureSensor::
AnalogPressureSensor(PinName pin, float calibration_m, float calibration_b) : 
AnalogInput(pin) {
	m = calibration_m;
	b = calibration_b;
}

AnalogPressureSensor::~AnalogPressureSensor() {
	
}


/*
 *Returns the current depth of the submarine, as determined by the linear equation
 * with parameters m and b (y = mx + b)
 * x is the raw pressure sensor reading (voltage)
 * y = calibrated pressure sensor reading (feet)
 * We add 0.5 to y so that it rounds correctly when we truncate into an integer.
 */
int AnalogPressureSensor::getValueCalibrated() {
	return (int)(getValueRaw() * m + b + 0.5f);
}

AnalogKillSwitch::AnalogKillSwitch(PinName Vin, PinName Vout, float thresh) :
AnalogInput(Vin) {
	threshold = thresh;
	p_Vout = new AnalogOut(Vout);
	p_Vout->write(1.0f); // 1.0 * 3.3 volts
}

AnalogKillSwitch::~AnalogKillSwitch() {
	// TODO: does the AnalogOut class automatically change the voltage to 0 when
	// it gets destroyed?
	// p_Vout->write(0.0f);
	if (p_Vout != NULL) {
		delete p_Vout;
	}
}

/* 
 * getValueRaw may not do what you think it does. The value is "raw" because it
 * is not thresholded. It is not "raw" because it is the raw data from the
 * sensor.
 */
float AnalogKillSwitch::getValueRaw() {
	// TODO: test this averaging code. Is it enough?
	value = (AnalogInput::getValueRaw() + value) / 2;
	return value;
}

/* 
 * Returns true if the raw value exceeds the threshold passed into the constructor
 */
bool AnalogKillSwitch::getValueThresh() {
	return (getValueRaw() > threshold);
}
