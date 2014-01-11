#include "avnavcontrol.h"
#include <stdio.h>

volatile int desHead(0), desDepth(2), desPower(100);
float acc_x(0), acc_y(0), acc_z(0);

float imu_data[9];
unsigned char motorArray[4];

void debug_mode();
void pid_tuning_mode();

LocalFileSystem local("local");
char mode, which_pid;

int main()
{
	init_pid();
	
	FILE* config = fopen("/local/config", "r");
	mode = fgetc(config);
	debug = (mode != 'R');
	if (mode == 'P')
	{
		which_pid = fgetc(config);
	}
	fclose(config);
	//attach the pc interrupts
	pc.p_device->attach(&rx_interrupt_pc, Serial::RxIrq);
	pc.p_device->attach(&tx_interrupt_pc, Serial::TxIrq);
	
	//attach motor interrupt
	motor.p_device->attach(&tx_interrupt_motor, Serial::TxIrq);
	
	//attach imu interrupt
	imu.p_device->attach(&rx_interrupt_imu, Serial::RxIrq);
	
	//create the tickers here
	Ticker tick[5];
	if (!debug)
	{
		tick[0].attach(&send_status_pc, 0.1);
	}
	tick[1].attach(&do_pid, DT);
	tick[2].attach(&motor_send_wrapper, DT);
	tick[3].attach(&updateKill, DT);	//cannot run faster than PID so that PID knows when to reset
	tick[4].attach(&updatePressure, 0.1);
	
	while (true)
	{
		
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
		if (!debug)
		{
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
		else
		{
			if (mode == 'D')
			{
				debug_mode();
			}
			if (mode == 'P')
			{
				pid_tuning_mode();
			}
		}
	}
}

void imu_calibration();
void imu_direct_access();
void kill_info();
void pressure_info();

void debug_mode()
{
	pc.send_message("Welcome to debug mode\r\n");
	tx_interrupt_pc();
	char in;
	while (true)
	{
		// Get the character from the PC.
		in = pc.readPC();
		switch (in)
		{
		case 'c':
			imu_calibration();
			break;
			
		case 'i':	// Act as a passthrough between IMU and PC
			imu_direct_access();
			break;
				
		case 'k':	// Print kill switch data.
			kill_info();
			break;
		
		case 'p':
			pressure_info();
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
	int imuCalibrationWait = 1000*10;
	Timer* time = new Timer();
	time->start();
	// Buffer for printing the IMU's calibration progress.
	char calibProgressMsg[30];
	// Previous time that the calibration message was printed.
	// Initialize to 0 so we don't print a message for time = 0.
	int lastTime = 0;
	// Loop until the calibration time is exceeded.
	while (time->read_ms() < imuCalibrationWait)
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
	char calibration_message[200];
	// Print a formatted message to the string.
	sprintf(
	 calibration_message,
	 "\t%f\t%f\t%f\r\n\t%f\t%f\t%f\r\n\t%f\t%f\t%f\r\n",
	 imu.sumAccX/(double)imu.num, imu.sumAccY/(double)imu.num, imu.sumAccZ/(double)imu.num,
	 imu.sumGyrX/(double)imu.num, imu.sumGyrY/(double)imu.num, imu.sumGyrZ/(double)imu.num,
	 imu.sumMagX/(double)imu.num, imu.sumMagY/(double)imu.num, imu.sumMagZ/(double)imu.num
	);

	//printf("IMU Calibration for the last %d ms and %d readings:\r\n%s", *imuCalibrationWait, imu.num, calibration_message);
	
	//Write the calibration message to a log file.
	FILE* calibration_file = fopen("/local/CALIBRAT", "w");
	fprintf(calibration_file, "%s", calibration_message);
	fclose(calibration_file);	

	// Safely send the string to the PC.
	pc.send_message(calibration_message);
	tx_interrupt_pc();
	// Tell the IMU to stop sending calibration.
	imu.setCalibrationEnabled(false);
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
	while (true)
	{
		// Fill the buffer with the message
		sprintf(killinfo, "kill: %f\n\r", kill.getValueRaw());
		// Safely send the buffer to the PC.
		pc.send_message(killinfo);
		// Make it flush the PC buffer.
		tx_interrupt_pc();
		// Wait 1s between each message.
		wait_ms(100);
	}
}

void pressure_info()
{
	char pressinfo[20];
	int count = 0;
	float average = 0;
	motor.set(FRONT, 0);
	motor.set(BACK, 254);
	motor.set(LEFT, 127);
	motor.set(RIGHT, 127);
	tx_interrupt_motor();
	
	while (true)
	{
		count++;
		average+=pressure.getValueRaw();
		if (count % 20 == 0)
		{
			sprintf(pressinfo, "pressure: %f\n\r", average/20);
			pc.send_message(pressinfo);
			tx_interrupt_pc();
			average = 0;
		}
		wait_ms(50);
	}
}

void set_gains();
void set_setpoint();
void print_gains();
void print_setpoint();
void print_status();

void pid_tuning_mode()
{
	desHead = 0;
	desPower = 100;
	desDepth = 25;
	/*
	pc.send_message("Entered PID tuning.\r\n");
	pc.send_message("g: set gains\r\n");
	pc.send_message("s: set setpoint\r\n");
	pc.send_message("p: print gains.\r\n");
	pc.send_message("d: view status\r\n");
	tx_interrupt_pc();
	*/
	
	Ticker status;
	status.attach(&print_status, 0.1);
	
	while (true)
	{
		if (!motor.isTxEmpty())
		{
			tx_interrupt_motor();
		}
		if (!pc.isTxEmpty())
		{
			tx_interrupt_pc();
		}
		imu.getData();
		desDepth = 10;
		give_data(imu.accX, imu.accY, imu.accZ, imu.gyrX, imu.gyrY, imu.gyrZ);
		for (int i = 0; i < 4; i++)
		{
			motor.set(i, motorArray[i]);
		}
		
		char mes = pc.readPC();
		switch (mes)
		{
		case 'g':	// Set gains
			set_gains();
			break;
			
		case 's':	//set setpoint
			set_setpoint();
			break;
				
		case '\0':
			break;
		}
		tx_interrupt_pc();
	}
}

void set_gains()
{
	//4 integers for each p i and d. divides what you give by 1000. leading zeroes if necessary
	float gains[3] = {0, 0, 0};
	for (int g = 0; g < 3; g++)
	{
		for (int i = 0; i < 4; i++)
		{
			char j;
			while (!(j = pc.readPC()));
			if (j < '0' || j > '9')
			{
				i--;
				continue;
			}
			gains[g] = gains[g] * 10 + (j - '0');
		}
		gains[g] /= 1000;
	}
	
	PID* activePID;
	switch (which_pid)
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
	
	return;
}

void set_setpoint()
{
	float point = 0;
	for (int i = 0; i < 3; i++)
	{
		char j;
		while (!(j = pc.readPC()));
		if (j < '0' || j > '9')
		{
			i--;
			continue;
		}
		point = 10 * point + (j - '0');
	}
	
	PID* activePID;
	switch (which_pid)
	{
	case 'p':
		activePID = pitchPID;   break;
	case 'd':
		activePID = depthPID; desDepth = point; break;
	case 'h':
		activePID = headingPID; desHead = point; break;
	}
	activePID->setSetpoint(point);
}

void print_status()
{
	char buff[25];
	
	int error;
	switch (which_pid)
	{
	case 'p': error = (calcP/45) * 100 + 100; break;
	case 'd': error = (desDepth - depth) * 100 / 120 + 100; break;
	case 'h':
		error = calcH - desHead;
		if (error > 180)
		{
			error -= 360;
		}
		if (error < -180)
		{
			error += 360;
		}
		error = ((float)error) / 180 * 100 + 100;
		break;
	}
	
	sprintf(buff, "%d\n", error);
	pc.send_message(buff);
	tx_interrupt_pc();
}
