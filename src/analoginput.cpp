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
