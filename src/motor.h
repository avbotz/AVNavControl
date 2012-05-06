#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"
#include <string>
#include <sstream>
#include "debug.h"

#define SYNC_BYTE 255

//ifndef used because it is defined in
//two seperate places and im not too sure
//who includes who
#ifndef LEFT
#define LEFT 0
#define RIGHT 1
#define FRONT 2
#define BACK 3
#endif

// Stores values for and communicates with the motor controller.
// We use the Mini SSC II format. This is used to control the motors and move the sub.
class Motor {
public:

	Motor(int num_motors, int baud, PinName tx, PinName rx);
	~Motor();
	void send(); // send all motors' speeds to the controller
	void send(int i_motor); // send speed of motor i_motor to the controller
	void set(unsigned char value); // set and send all the motors to a single value
	void set(int i_motor, unsigned char value); // set and send motor number i_motor to value
	char get(int i_motor); // get the current speed for motor number i_motor
private:
	Serial* p_device; // pointer to our serial object (used to send stuff)
	int num_motors; // number of motors, counting from 1

	/*
	 * These arrays send input to the controller which in turn sends to the motors.
	 * The format for the arrays: {SYNC_BYTE, servo number, speed}
	 * SYNC_BYTE tells the controller that we're about to send new instructions.
	 * The servo number picks the motor. 0 = right, 1 = back, 2 = front, 3 = left
	 * We set the speed anywhere between 0 and 254. 127 is off, 254 is 100% forward, 0 is 100% back.
	 * On the back motor, the numbers are reversed. 0 is 100% forward, 254 is 100% back. 127 is still off.
	 * The speed scales approximately linearly with the set values, though there is no motor motion until around 127+-15.
	 */

	char motors[12]; //The controller supports twelve motors. This array locally stores their speeds.
};

extern Motor motor;

#endif
