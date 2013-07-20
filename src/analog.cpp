#include "analog.h"

#include "pc.h"
extern PC pc;

AnalogInput::AnalogInput(PinName pin) {
	p_analog = new AnalogIn(pin);
}

AnalogInput::~AnalogInput() {
	if (p_analog != NULL) {
		delete p_analog;
	}
}

// Returns the raw value of the device. This is a noisy value and probably
// shouldn't be used unless you actually want the noisy value.
float AnalogInput::getValueRaw() {
	return p_analog->read();
}

void AnalogPressureSensor::changeB(float new_b)
{
	b = new_b;
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

void updatePressure() {
	depth = pressure.getValueCalibrated();
}

// The AnalogKillSwitch class basically sets up two pins on the mbed to act like
// an ohmmeter.
AnalogKillSwitch::AnalogKillSwitch(PinName Vin, PinName Vout, float thresh) :
AnalogInput(Vin) {
	threshold = thresh;
	value = thresh;
	p_Vout = new AnalogOut(Vout);
	// Put 1.0f * 3.3 volts on the kill voltage out (Vout) pin.
	p_Vout->write(1.0f);
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

void updateKill() {
	isAlivePrev = isAlive;
	isAlive = kill.getValueThresh();
}
