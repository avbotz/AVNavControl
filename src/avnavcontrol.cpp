#include "avnavcontrol.h"

int desHead(0), desDepth(2), desPower(100);
float acc_x(0), acc_y(0), acc_z(0);
float depth(0.0f);
bool manual = false;

unsigned char motorArray[4];


int main() {

	//
	init_pid();

	// poll devices
	// imu
	// kill & pressure
	// kalman
	// pid

	// every 1s: send data to pc
	// upon serial from pc: set desired heading and desired depth
	//print_serial(&pc, "hi");
	
	while (true) {
		//led1 = isAlive = kill.getValueThresh();
		//led2 = !led2;
		if (true) {

			if (!debug) {
				pc.readPC();
				desHead = pc.desired_heading;
				desPower = pc.desired_power;
				desDepth = pc.desired_depth;
			}

			imu.getData();
	
			led3 = !led3;

			if (!manual) {
				give_data(imu.accX, imu.accY, imu.accZ, imu.gyrX, imu.gyrY, imu.gyrZ);
				for (int i = 0; i < 4; i++) {
					motor.set(i, motorArray[i]);
				}
			}
			if (motor.buffer_empty) {
				tx_interrupt_motor();
			}
			if (pc.tx_empty) {
				tx_interrupt_pc();
			}
	
			if (debug) {
				char in;
				while (in = pc.readPC()) {
					// TODO: this should be used to handle message from AVI
					// Format for messages from AVI is com prefix, data byte 1, data byte 2, '\n'
					// where com prefix is h for desired heading, d for desired depth,
					// p for desired power
					manual = true;
					switch (in) {
						case 'g':		// All motors
							motor.set(LEFT, 0);
							motor.set(RIGHT, 0);
							motor.set(FRONT, 0);
							motor.set(BACK, 254);
							break;
						case 'f':		// Forward
							motor.set(LEFT, 0);
							motor.set(RIGHT, 0);
							motor.set(FRONT, 127);
							motor.set(BACK, 127);
							break;
						case 'b':		// Back
							motor.set(LEFT, 254);
							motor.set(RIGHT, 254);
							motor.set(FRONT, 127);
							motor.set(BACK, 127);
							break;
						case 'u':		// Up
							motor.set(LEFT, 127);
							motor.set(RIGHT, 127);
							motor.set(FRONT, 254);
							motor.set(BACK, 0);
							break;
						case 'd':		// Down
							motor.set(LEFT, 127);
							motor.set(RIGHT, 127);
							motor.set(FRONT, 0);
							motor.set(BACK, 254);
							break;
						case 'r':		// Turn right
							motor.set(LEFT, 0);
							motor.set(RIGHT, 254);
							motor.set(FRONT, 127);
							motor.set(BACK, 127);
							break;
						case 'l':		// Turn left
							motor.set(LEFT, 254);
							motor.set(RIGHT, 0);
							motor.set(FRONT, 127);
							motor.set(BACK, 127);
							break;
						case 'z':		// Debug; runs the back motor
							motor.set(LEFT, 127);
							motor.set(RIGHT, 127);
							motor.set(FRONT, 127);
							motor.set(BACK, 254);
							break;
						case 'y':		// Debug: runs all but the left motor
							motor.set(LEFT, 255);
							motor.set(RIGHT, 0);
							motor.set(FRONT, 0);
							motor.set(BACK, 0);
							break;
						case 's':		// Stops the motors immediately
							motor.set(LEFT, 127);
							motor.set(RIGHT, 127);
							motor.set(FRONT, 127);
							motor.set(BACK, 127);
								break;
						case 'i':
							manual = false;
							//print info and stuff
							break;
						case '\e': // escape
							manual = false;
							break;
						case '9':	// Act as a passthrough between IMU and PC
							imu.directAccess();
							manual = false;
							break;
						default:
							pc.send_message("Unrecognized command.\n\r");
						break;
					}
				}
			}
	
		}
		else {
			motor.set(127);
		}
	}

}

