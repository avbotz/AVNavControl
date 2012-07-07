#include "avnavcontrol.h"

int desHead(0), desDepth(2), desPower(100);
float acc_x(0), acc_y(0), acc_z(0);
volatile int depth(0);

unsigned char motorArray[4];


int main() {

	init_pid();

	//attach the pc interrupts
	pc.p_device->attach(&rx_interrupt_pc, Serial::RxIrq);
	pc.p_device->attach(&tx_interrupt_pc, Serial::TxIrq);

	//attach motor interrupt
	motor.p_device->attach(&tx_interrupt_motor, Serial::TxIrq);

	//attach imu interrupt
	imu.p_device->attach(&rx_interrupt_imu, Serial::RxIrq);


	//create the tickers here
	Ticker tick[3];
	if (!debug) {
		tick[0].attach(&send_status_pc, 1.0);
	}
	tick[1].attach(&do_pid, DT);
	tick[2].attach(&motor_send_wrapper, DT/2);

	// poll devices
	// imu
	// kill & pressure
	// kalman
	// pid

	// every 1s: send data to pc
	// upon serial from pc: set desired heading and desired depth
	
	while (true) {
		led3 = !led3;

		if (motor.buffer_empty) {
			tx_interrupt_motor();
		}
		if (pc.tx_empty) {
			tx_interrupt_pc();
		}		
		//if (kill.getValueThresh()) continue;
		//led1 = isAlive = kill.getValueThresh();
		//led2 = !led2;
		imu.getData();
		if (!debug) {
			pc.readPC();
			desHead = pc.desired_heading;
			desPower = pc.desired_power;
			desDepth = pc.desired_depth;
			give_data(imu.accX, imu.accY, imu.accZ, imu.gyrX, imu.gyrY, imu.gyrZ);
			for (int i = 0; i < 4; i++) {
				motor.set(i, motorArray[i]);
			}
		}		
		else {
			char in;
			while ((in = pc.readPC())) {
				switch (in) {
					case 'c':
					{ // Braces here so that variables initialized in this case aren't visible from other cases.
						pc.send_message("Begin IMU calibration.\n");
						tx_interrupt_pc();
						// Tell the IMU to start saving calibration information.
						imu.setCalibrationEnabled(true);
						// Give it some time.
						int* imuCalibrationWait = new int;
						*imuCalibrationWait = 1000 * 10;
						Timer* time = new Timer();
						time->start();
						while (time->read_ms() < *imuCalibrationWait) {
							if (!(time->read_ms() % 10)) {
								imu.getData();
							}
						}
						// String to store the information message for PC.
						char* calibration_message = new char[200];
						// Print a formatted message to the string.
						sprintf(
						 calibration_message,
						 "IMU averages for last %d ms and %d readings: %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
						 *imuCalibrationWait, imu.num,
						 imu.sumAccX/(double)imu.num, imu.sumAccY/(double)imu.num, imu.sumAccZ/(double)imu.num,
						 imu.sumGyrX/(double)imu.num, imu.sumGyrY/(double)imu.num, imu.sumGyrZ/(double)imu.num,
						 imu.sumMagX/(double)imu.num, imu.sumMagY/(double)imu.num, imu.sumMagZ/(double)imu.num
						);
						// Safely send the string to the PC.
						pc.send_message(calibration_message);
						// Tell the IMU to stop sending calibration.
						imu.setCalibrationEnabled(false);
						// Free the memory used by the string.
						delete calibration_message;
						delete imuCalibrationWait;
						delete time;
						break;
					}
					case 'i':	// Act as a passthrough between IMU and PC
						imu.directAccess();
						break;
					case 'k':
					{
						char killinfo[20];
						while (true) {
							sprintf(killinfo, "kill: %f\n\r", kill.getValueRaw());
							pc.send_message(killinfo);
							tx_interrupt_pc();
							wait_ms(1000);
						}
						break;
					}
					default:
						pc.send_message("Unrecognized command.\n\r");
					break;
				}
			}
		}
	}
}
