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

	for (int i = 0; i < MOTOR_TX_BUF_SIZE; i++) {
		buffer[i] = 0;
	}
	i_buffer_write = i_buffer_read = 0;
	buffer_empty = true;
	set(127);   // The motors should start stationary (zero power)
}

// MiniSSC2's Destructor
Motor::~Motor() {
	if (p_device != NULL) {
		// must do this. otherwise, you'll have memory leakage & you may not be able to re-open the serial port later
		delete p_device;
	}
}

void Motor::putc(char c) {
	buffer[i_buffer_write] = c;
	NVIC_DisableIRQ(UART1_IRQn);
	i_buffer_write = (i_buffer_write + 1) % MOTOR_TX_BUF_SIZE;
	NVIC_EnableIRQ(UART1_IRQn);
	// Don't worry about overflow because if you're 1024 chars behind you're FUBAR already
}

void motor_send_wrapper() {
	//if (!kill.getValueThresh()) return;
	motor.send();
}

void Motor::send() {
	for (int i = 0; i < num_motors; i++) {
		send(i);
	}
}

void Motor::send(int i_motor) {
	// format: {sync byte, motor id, motor power}
	// example: {SYNC_BYTE, 2, 24} sets motor 2 to power level 24
	putc(SYNC_BYTE);
	putc((unsigned char)i_motor);
	putc(motors[i_motor]);
}

// Minus 1 to stop dropper from turing into 127 (drop marker state)
void Motor::set(unsigned char value) {
	for (int i = 0; i < (num_motors-1); i++) {
		set(i, value);
	}
	set(4, (unsigned char)0);
}

void Motor::set(int i_motor, unsigned char value) {
	motors[i_motor] = value;
}

char Motor::get(int i_motor) {
	return motors[i_motor];
}

void tx_interrupt_motor() {
	while (motor.p_device->writeable() && motor.i_buffer_write != motor.i_buffer_read) {
		motor.p_device->putc(motor.buffer[motor.i_buffer_read]);
		motor.i_buffer_read = (motor.i_buffer_read + 1) % MOTOR_TX_BUF_SIZE;
		motor.buffer_empty = false;
	}
	if (motor.i_buffer_write == motor.i_buffer_read) {
		motor.buffer_empty = true;
		NVIC_DisableIRQ(UART1_IRQn);
		// if nothing to write, turn off the interrupt until motor.getc() is called again
	}
}


// Don't delete this comment
