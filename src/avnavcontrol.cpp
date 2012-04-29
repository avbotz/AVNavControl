#include "avnavcontrol.h"

int desHead(0), desDepth(2), desPower(100);
float acc_x(0), acc_y(0), acc_z(0);
float depth(0.0f);
bool manual = false;

unsigned char motorArray[4];

int main() {
	// Set BeagleBoard's baud to 115200.
	pc.baud(115200);
	// Attach a callback so that readIMU() is called when the IMU gets a character.
	imu.attach(&readIMU);
	// Set the motors to no power.
	motor.set(127);
	// Set an interrupt to send data to the BeagleBoard every 1.0 seconds.
	ticker_pc.attach(&sendPC, 1.0);
	//
	init_pid();

	// poll devices
	// imu
	// kill & pressure
	// kalman
	// pid

	// every 1s: send data to pc
	// upon serial from pc: set desired heading and desired depth

	while (true) {
		led1 = isAlive = kill.getValueThresh();
		if (isAlive) {
			imu.getData();
			//pc.printf("got data\n\r");
	
			if (!manual) {
				give_data(imu.accX, imu.accY, imu.accZ, imu.gyrX, imu.gyrY, imu.gyrZ);
				do_pid();
				//pc.printf("Motor speeds:	");
				for (int i = 0; i < 4; i++) {
					//motor.set(i, motorArray[i]);
					//pc.printf("%d\t", motorArray[i]);
				}
				//pc.printf("\n\r");
			}
	
			while (pc.readable()) {
				// TODO: this should be used to handle message from AVI
				// Format for messages from AVI is com prefix, data byte 1, data byte 2, '\n'
				// where com prefix is h for desired heading, d for desired depth,
				// p for desired power
				manual = true;
				char in;
				in = pc.getc();
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
						manual = true;
						//print info and stuff
						break;
					case '\e': // escape
						manual = false;
						break;
					default:
						pc.printf("Unrecognized command.\n\r");
						break;
	
				}
	
				//read stuff here
				//PCreadable = false;
				//NVIC_EnableIRQ(UART0_IRQn);
			}
			
			// Update the BeagleBoard with data.
			if (PCsendable) {
				NVIC_DisableIRQ(UART0_IRQn);
				send_to_pc();
				PCsendable = false;
				NVIC_EnableIRQ(TIMER3_IRQn);
				NVIC_EnableIRQ(UART0_IRQn);
				//__enable_irq();
			}
	
		}
		else {
			motor.set(127);
		}
	}

}

// This interrupt is called when there is a character to be read from the PC. To
// avoid a crash or livelock, we disable interrupts at the end of the interrupt.
// Then, in the main loop, we read everything from the buffer.
void readPC() {
	//PCreadable = true;
	//NVIC_DisableIRQ(UART0_IRQn);
}

// This interrupt is called when there is a character to be read from the IMU.
void readIMU() {
	imu.IMUreadable = true;
	NVIC_DisableIRQ(UART3_IRQn);
}

void sendPC() {
	//PCsendable = true;
	//NVIC_DisableIRQ(TIMER3_IRQn);
	//__disable_irq();
}
