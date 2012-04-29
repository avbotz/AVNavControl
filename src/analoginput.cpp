#include "analoginput.h"

AnalogInput::AnalogInput(PinName pin, float thresh) {
	threshold = thresh;
	p_analog = new AnalogIn(pin);
}

AnalogInput::~AnalogInput() {
	if (p_analog != NULL) {
		delete p_analog;
	}
}

float AnalogInput::getValueRaw() {
	return p_analog->read();
}

bool AnalogInput::getValueThresh() {
	return (p_analog->read() > threshold);
}

AnalogPressureSensor::
AnalogPressureSensor(PinName pin, float calibration_m, float calibration_b) : 
AnalogInput(pin, 0.5f) {
	m = calibration_m;
	b = calibration_b;
}

AnalogPressureSensor::~AnalogPressureSensor() {
	
}

int AnalogPressureSensor::getValueCalibrated() {
	// This is a y = mx + b equation.
	// x = raw pressure sensor reading (voltage)
	// y = calibrated pressure sensor reading (feet)
	// We're adding 0.5 so that it rounds correctly when we truncate into an int.
	return (int)(getValueRaw() * m + b + 0.5f);
}

AnalogKillSwitch::
AnalogKillSwitch(PinName Vin, PinName Vout, float thresh) :
AnalogInput(Vin, thresh) {
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

float AnalogKillSwitch::getValueRaw() {
	// TODO: test this averaging code. Is it enough?
	value = (AnalogInput::getValueRaw() + value) / 2;
	return value;
}

bool AnalogKillSwitch::getValueThresh() {
	return (value > threshold);
}