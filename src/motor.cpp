/*
 * Demo that communicates with the MiniSSC 2. We should eventually refactor this,
 * as we have replaced the MiniSSC 2 with a Pololu Maestro. BAM REFACTORED BITCHES
 */

#include "mbed.h"
#include "motor.h"

// MiniSSC2's Constructor
Motor::Motor(int num_motors, int baud, PinName tx, PinName rx)
{

	this->num_motors = num_motors;
	p_device = new Serial(tx, rx); // (tx, rx) opens up new serial device (p_device is Serial* pointer)
	// Settings for Mini Maestro serial -- see Mini Maestro data sheet
	p_device->format(8, Serial::None, 1);
	p_device->baud(baud); // Set the baud.
	tx_buffer = new CircularBuffer(MOTOR_TX_BUF_SIZE);
	
	set(127);   // The motors should start stationary (zero power)
}

// MiniSSC2's Destructor
Motor::~Motor()
{
	if (p_device != NULL) {
		// must do this. otherwise, you'll have memory leakage & you may not be able to re-open the serial port later
		delete p_device;
	}
	delete tx_buffer;
}

void Motor::putc(char c)
{
	NVIC_DisableIRQ(UART2_IRQn);
	tx_buffer->writeByte(c);
	NVIC_EnableIRQ(UART2_IRQn);
	// Don't worry about overflow because if you're 1024 chars behind you're FUBAR already
}

void motor_send_wrapper()
{
	motor.send();
}

void Motor::send()
{
	for (int i = 0; i < num_motors; i++)
	{
		send(i);
	}
}

void Motor::send(int i_motor)
{
	// format: {sync byte, motor id, motor power}
	// example: {SYNC_BYTE, 2, 24} sets motor 2 to power level 24
	putc(SYNC_BYTE);
	putc((unsigned char)i_motor);
	putc(motors[i_motor]);
}

void Motor::set(unsigned char value)
{
	for (int i = 0; i < num_motors; i++)
	{
		set(i, value);
	}
}

void Motor::set(int i_motor, unsigned char value)
{
	motors[i_motor] = value;
}

char Motor::get(int i_motor) const
{
	return motors[i_motor];
}

bool Motor::isTxEmpty() const
{
	return motor.tx_buffer->empty;
}

void tx_interrupt_motor()
{
	while (motor.p_device->writeable() && !motor.tx_buffer->empty)
	{
		motor.p_device->putc(motor.tx_buffer->readByte());
	}
	if (motor.tx_buffer->empty)
	{
//		NVIC_DisableIRQ(UART2_IRQn);
		// if nothing to write, turn off the interrupt until motor.getc() is called again
	}
}
