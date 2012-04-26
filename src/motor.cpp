/*
 * Demo that communicates with the MiniSSC 2. We should eventually refactor this,
 * as we have replaced the MiniSSC 2 with a Pololu Maestro. BAM REFACTORED BITCHES
 */

#include "mbed.h"
#include "motor.h"

// MiniSSC2's Constructor
Motor::Motor(int num_motors, int baud, PinName tx, PinName rx) {
	this->num_motors = num_motors;
	p_device = new Serial(tx, rx); // (tx, rx) opens up new serial device (p_device is Serial* pointer)
	p_device->format(8, Serial::None, 1);
	p_device->baud(baud); // Set the baud.
	set(127);   // The motors should start stationary (zero power)
}

// MiniSSC2's Destructor
Motor::~Motor() {
	if (p_device != NULL) {
		// must do this. otherwise, you'll have memory leakage & you may not be able to re-open the serial port later
		delete p_device;
	}
}

void Motor::send() {
	for (int i = 0; i < num_motors; i++) {
		send(i);
	}
}

void Motor::send(int i_motor) {
	// format: {sync byte, motor id, motor power}
	// example: {SSC_SYNC_BYTE, 2, 24} sets motor 2 to power level 24
	p_device->putc(SYNC_BYTE);
	p_device->putc((unsigned char)i_motor);
	p_device->putc(motors[i_motor]);
}

void Motor::set(unsigned char value) {
	for (int i = 0; i < num_motors; i++) {
		set(i, value);
	}
}

void Motor::set(int i_motor, unsigned char value) {
	motors[i_motor] = value;
	send(i_motor);
}

char Motor::get(int i_motor) {
	return motors[i_motor];
}


// Don't delete this comment
