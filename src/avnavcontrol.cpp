#include "avnavcontrol.h"

volatile int desHead(0), desDepth(2), desPower(100);
float acc_x(0), acc_y(0), acc_z(0);

unsigned char motorArray[4];

void debug_mode();
void imu_calibration();
void imu_direct_access();
void kill_info();
void pid_tuning();
void set_gains();
void set_setpoint();
void print_gains();
void print_setpoint();
void print_status();

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
	Ticker tick[5];
	if (!debug) {
		tick[0].attach(&send_status_pc, 0.1);
	}
	tick[1].attach(&do_pid, DT);
	tick[2].attach(&motor_send_wrapper, DT/2);
	tick[3].attach(&updateKill, .01);
	tick[4].attach(&updatePressure, 0.1);
	
	while (true) {
		if (!motor.isTxEmpty()) {
			tx_interrupt_motor();
		}
		if (!pc.isTxEmpty()) {
			tx_interrupt_pc();
		}
		
		// Set led1 to the current kill state.
		led1 = isAlive;
		
		imu.getData();
		
		// If we are running in normal (AKA production or competition) mode.
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
		// Otherwise, we are running in debug mode.
		else {
			debug_mode();
		}
	}
}


void debug_mode()
{
	char in;
	while (true) {
		// Get the character from the PC.
		in = pc.readPC();
		switch (in) {
		case 'c':
			imu_calibration();
			break;
			
		case 'i':	// Act as a passthrough between IMU and PC
			imu_direct_access();
			break;
				
		case 'k':	// Print kill switch data.
			kill_info();
			break;
			
		case 'p':	// PID tuning
			pid_tuning();
			break;
				
		case '\0':
			break;
				
		default:	// The user typed a key we didn't understand.
			pc.send_message("Unrecognized command.\n\r");
			if (!pc.isTxEmpty())
			{
				tx_interrupt_pc();
			}
			break;
		}
	}
}

void imu_calibration()
{
	pc.send_message("Begin IMU calibration.\r\n");
	tx_interrupt_pc();
	// Tell the IMU to start saving calibration information.
	imu.setCalibrationEnabled(true);
	// Give it some time.
	int* imuCalibrationWait = new int;
	*imuCalibrationWait = 1000 * 10;
	Timer* time = new Timer();
	time->start();
	// Buffer for printing the IMU's calibration progress.
	char calibProgressMsg[30];
	// Previous time that the calibration message was printed.
	// Initialize to 0 so we don't print a message for time = 0.
	int lastTime = 0;
	// Loop until the calibration time is exceeded.
	while (time->read_ms() < *imuCalibrationWait)
	{
		// Get the elapsed time from timer.
		int timeElapsed = time->read_ms();
		// Get data every 10 ms.
		if (!(timeElapsed % 10))
		{
			imu.getData();
		}
		// If the time is a multiple of 1000 ms and we haven't already printed at this time.
		if (timeElapsed % 1000 == 0 && timeElapsed != lastTime)
		{
			// Print the calibration progress.
			sprintf(calibProgressMsg, "Calibrating: %d readings in %d ms\r\n", imu.num, timeElapsed);
			pc.send_message(calibProgressMsg);
			tx_interrupt_pc();
			// Save the elapsed time this message was printed at.
			lastTime = timeElapsed;
		}
	}
	// String to store the information message for PC.
	char* calibration_message = new char[200];
	// Print a formatted message to the string.
	sprintf(
	 calibration_message,
	 "IMU averages for last %d ms and %d readings:\r\n\t%f,\t%f,\t%f\r\n\t%f,\t%f,\t%f\r\n\t%f,\t%f,\t%f\r\n",
	 *imuCalibrationWait, imu.num,
	 imu.sumAccX/(double)imu.num, imu.sumAccY/(double)imu.num, imu.sumAccZ/(double)imu.num,
	 imu.sumGyrX/(double)imu.num, imu.sumGyrY/(double)imu.num, imu.sumGyrZ/(double)imu.num,
	 imu.sumMagX/(double)imu.num, imu.sumMagY/(double)imu.num, imu.sumMagZ/(double)imu.num
	);
	// Safely send the string to the PC.
	pc.send_message(calibration_message);
	tx_interrupt_pc();
	// Tell the IMU to stop sending calibration.
	imu.setCalibrationEnabled(false);
	// Free the memory used by the string.
	delete calibration_message;
	delete imuCalibrationWait;
	delete time;
}

void imu_direct_access()
{
	imu.directAccess();
}
void kill_info()
{
	// Buffer to hold the message.
	char killinfo[20];
	// Loop until reset
	while (true) {
		// Fill the buffer with the message
		sprintf(killinfo, "kill: %f\n\r", kill.getValueRaw());
		// Safely send the buffer to the PC.
		pc.send_message(killinfo);
		// Make it flush the PC buffer.
		tx_interrupt_pc();
		// Wait 1s between each message.
		wait_ms(1000);
	}
}

void pid_tuning()
{
	desHead = 0;
	desPower = 100;
	desDepth = 25;
	
	pc.send_message("Entered PID tuning.\r\n");
	pc.send_message("g: set gains\r\n");
	pc.send_message("s: set setpoint\r\n");
	pc.send_message("p: print gains.\r\n");
	pc.send_message("d: view status\r\n");
	tx_interrupt_pc();
	
	while (true) {
		if (!motor.isTxEmpty()) {
			tx_interrupt_motor();
		}
		if (!pc.isTxEmpty()) {
			tx_interrupt_pc();
		}
		imu.getData();
		give_data(imu.accX, imu.accY, imu.accZ, imu.gyrX, imu.gyrY, imu.gyrZ);
		for (int i = 0; i < 4; i++) {
			motor.set(i, motorArray[i]);
		}
		
		char mes = pc.readPC();
		switch (mes) {
		case 'g':	// Set gains
			set_gains();
			break;
			
		case 's':	//set setpoint
			set_setpoint();
			break;
			
		case 'p':
			print_gains();
			break;
			
		case 'q':
			print_setpoint();
			break;
			
		case 'd':
			print_status();
			break;
				
		case '\0':
			break;
			
		default:
			pc.send_message("what the hell were you thinking?\r\n");
		}
		tx_interrupt_pc();
	}
}

void set_gains()
{
	pc.send_message("Setting gains. Which one? ");
	tx_interrupt_pc();
	
	// Block while waiting for next character
	char mes2;
	while (!(mes2 = pc.readPC()));
	
	if (mes2 == 'p' || mes2 == 'd' || mes2 == 'h')
	{
		// Stop the motors
		motor.set(127);
		tx_interrupt_motor();
		//4 integers for each p i and d. divides what you give by 1000. leading zeroes if necessary
		float gains[3] = {0, 0, 0};
		for (int g = 0; g < 3; g++)
		{
			for (int i = 0; i < 4; i++)
			{
				char j;
				while (!(j = pc.readPC()));
				if (j < '0' || j > '9') {
					i--;
					continue;
				}
				gains[g] = gains[g] * 10 + (j - '0');
			}
			gains[g] /= 1000;
		}
		
		PID* activePID;
		switch (mes2)
		{
		case 'p':
			activePID = pitchPID; break;
		case 'd':
			activePID = depthPID; break;
		case 'h':
			activePID = headingPID; break;
		}
		activePID->setGains(gains[0], gains[1], gains[2]);
		activePID->reset();
		pc.send_message("gains set\r\n");
	}
	else
	{
		pc.send_message("no gains for that letter\n\r");
	}
}

void set_setpoint()
{
	pc.send_message("Setting setpoint. Which one? ");
	tx_interrupt_pc();
	char mes2;
	while (!(mes2 = pc.readPC()));	//block waiting for next character
	
	if (mes2 == 'p' || mes2 == 'd' || mes2 == 'h')
	{
		//3 integers for set point use leading zeroes if necessary
		motor.set(127);
		tx_interrupt_motor();	//kill motors
		float point = 0;
		for (int i = 0; i < 3; i++)
		{
			char j;
			while (!(j = pc.readPC()));
			if (j < '0' || j > '9') {
				i--;
				continue;
			}
			point = 10 * point + (j - '0');
		}
		
		PID* activePID;
		switch (mes2)
		{
		case 'p':
			activePID = pitchPID;   break;
		case 'd':
			activePID = depthPID; desDepth = point; break;
		case 'h':
			activePID = headingPID; desHead = point; break;
		}
		activePID->setSetpoint(point);
		activePID->reset();
		
		pc.send_message("setpoint set\r\n");
	}
	else
	{
		pc.send_message("no setpoint for that letter\r\n");
	}
}

void print_gains()
{
	char output[100];
	
	sprintf(output, "Pitch gains: %f, %f, %f\r\nDepth gains: %f, %f, %f\r\nHeading gains: %f, %f, %f\r\n", 
			pitchPID->_kp, pitchPID->_ki, pitchPID->_kd,
			depthPID->_kp, depthPID->_ki, depthPID->_kd,
			headingPID->_kp, headingPID->_ki, headingPID->_kd);
	pc.send_message(output);
}

void print_setpoint()
{
	char out[50];
	sprintf(out, "Heading setpoint: %d\r\nDepth setpoint: %d\r\n", desHead, desDepth);
	pc.send_message(out);
}
void print_status()
{
	char buff[25];
	sprintf(buff, "Heading: %f\tDepth: %d\tPitch: %f\tKill: %d\r\n", calcH, depth, calcP, isAlive);
	pc.send_message(buff);
}