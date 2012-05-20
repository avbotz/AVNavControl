#include "avnavcontrol.h"

int desHead(0), desDepth(2), desPower(100);
float acc_x(0), acc_y(0), acc_z(0);
float depth(0.0f);
bool manual = false;

unsigned char motorArray[4];

Serial asdf(p28, p27);

int main() {
	// Set BeagleBoard's baud to 115200.
	pc.baud(115200);
	asdf.baud(9600);
	asdf.format(8, Serial::None, 1);
	// Attach a callback so that readIMU() is called when the IMU gets a character.
	imu.attach(&readIMU);
	imu.p_device->putc('4');	// tell the imu to start sending in case it isn't doing that already.
	// Set the motors to no power.
	motor.set(127);
	// Attach a callback for sending the motor.
	motor.attach(&sendMotor);
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
	//print_serial(&pc, "hi");
	int i = 0;
	while (true) {
		//led1 = isAlive = kill.getValueThresh();
		//led2 = !led2;
		if (true) {
			imu.getData();
			//pc.printf("got data\n\r");
	
			led3 = !led3;

			if (!manual) {
				give_data(imu.accX, imu.accY, imu.accZ, imu.gyrX, imu.gyrY, imu.gyrZ);
				do_pid();
				//pc.printf("Motor speeds:	");
				for (int i = 0; i < 4; i++) {
					//sprintf(filler, "%d\t", motorArray[i]);
					motor.set(i, motorArray[i]);
					//pc.printf("%d\t", motorArray[i]);
				}
			}
			if (motor.buffer_empty) {
				sendMotor();
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
	led2 = !led2;
	
	// TODO: it might be called and there are multiple characters to be read
	while (imu.p_device->readable()) {
		imu.buffer[imu.i_buffer_write] = imu.getc();
		//pc.putc(imu.buffer[imu.i_buffer_write]);
		NVIC_DisableIRQ(UART3_IRQn);
		imu.i_buffer_write = (imu.i_buffer_write + 1) % IMU_RX_BUFFER_SIZE;
		NVIC_EnableIRQ(UART3_IRQn);
		if (imu.i_buffer_write == imu.i_buffer_read) {
			imu.buffer_overflow = true;
			NVIC_DisableIRQ(UART3_IRQn);
			break;
		}
	}
}

void sendPC() {
	//PCsendable = true;
	//NVIC_DisableIRQ(TIMER3_IRQn);
	//__disable_irq();
}

void sendMotor() {
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