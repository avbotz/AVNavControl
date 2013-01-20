#include "mbed.h"
#include "imu.h"

IMU::IMU(PinName tx, PinName rx, int baud, PC* pc) {
	// Save a pointer to the PC for debugging use.
	p_pc = pc->p_device;
	p_device = new Serial(tx, rx);
	p_device->baud(baud);
	// serial settings for the IMU--refer to IMU's data sheet
	p_device->format(8, Serial::None, 1);
	
	// Calibration variables
	calibrationEnabled = false;
	
	accX = accY = accZ = gyrX = gyrY = gyrZ = magX = magY = magZ = 0;
	parseNow = false;
	
	//intialize the buffers and buffer indicies
	for (int i = 0; i < IMU_RX_BUFFER_SIZE; i++) {
		buffer[i] = 0;
	}
	i_buffer_write = i_buffer_read = 0;
	
	for (int i = 0; i < 1024; i++) {
		linebuf[i] = 0;
	}
	i_linebuf = 0;
	buffer_overflow = false;
	
	// Attach it as an RX interrupt only.
	
	// Tell the imu to start sending in case it isn't doing that already. (The
	// IMU sends data upon receiving the character '4'. Note that this is not
	// the numerical value 4.
	p_device->putc('4');
}

IMU::~IMU() {
	if (p_device != NULL) {
		delete p_device; // prevents memory leaks
	}
}

//Wrapper function to write a character to the IMU
void IMU::putc(char c) {
	p_device->putc(c);
}

// Checks data integrity of buffer, then stores the IMU values into variables.
void IMU::parse() {
	linebuf[i_linebuf] = '\0';
	//print_serial(p_pc, buf);
	short temp[9];
	int temp_temperature;
	// Check data integrity of sscanf()'s results. sscanf() returns the number
	// of values that it was able to extract. In this case, there are 9 values
	// we are reading.
	// we <3 pointer arithmetic
	if (sscanf(linebuf, "\n\r$%hd,%hd,%hd,%hd,%hd,%hd,%d,%hd,%hd,%hd#", 
		temp, temp+1, temp+2, temp+3, temp+4, temp+5, &temp_temperature, temp+6, temp+7, temp+8) == 9)
	{
		accX = temp[0];
		accY = temp[1];
		accZ = temp[2];
		gyrX = temp[3];
		gyrY = temp[4];
		gyrZ = temp[5];
		temperature = temp_temperature;
		magX = temp[6];
		magY = temp[7];
		magZ = temp[8];
		
		//calculate the actual temperature in degrees celcius from the reading
		//add the offset, divide by the scale, adjust the number to center around 35
		temperature = (temperature - 13200) / 280 + 35;
		
		// Indicate that we read data successfully.
		led3 = !led3;
	}
}


bool IMU::readable() {
	return p_device->readable();
}

void IMU::getData() {
	// While there is data to be read, or the buffer has overflowed.
	while (i_buffer_read != i_buffer_write || buffer_overflow) {
		linebuf[i_linebuf] = buffer[i_buffer_read];
		i_buffer_read = (i_buffer_read + 1) % IMU_RX_BUFFER_SIZE;
		buffer_overflow = false;
		i_linebuf++;
		if (linebuf[i_linebuf - 1] == '#') {
			parseNow = true;
			break;
		}
		led4 = !led4;
	}
	
	NVIC_EnableIRQ(UART3_IRQn);
	
	if (parseNow) {
		parse();
		
		/*
		 * IMU Calibration Code
		 * Calculates the average values of all sensors in order to set biases. 
		 * The IMU should be flat when you run this code.
		 */
		if (calibrationEnabled) {
			sumAccX += accX;
			sumAccY += accY;
			sumAccZ += accZ;

			sumGyrX += gyrX;
			sumGyrY += gyrY;
			sumGyrZ += gyrZ;

			sumMagX += magX;
			sumMagY += magY;
			sumMagZ += magZ;

			num++;
		}
		
		// Clear the line buffer.
		for (int i = 0; i < 1024; i++) {
			linebuf[i] = 0;
		}
		i_linebuf = 0;
		
		parseNow = false;
	}
}

// Call to get direct (raw) access to the IMU when it screws itself up.
void IMU::directAccess() {
	// Disable the interrupt for reading from the IMU
	NVIC_DisableIRQ(UART3_IRQn);
	
	p_pc->printf("Entering IMU direct access mode. To exit, you need to reset the mbed.\n\r");
	
	// Act as a passthrough between the IMU and the PC (BeagleBoard).
	while (true) {
		// Send a character from PC to IMU (if possible).
		if (p_pc->readable()) {
			p_device->putc(p_pc->getc());
		}
		// Send a character from IMU to PC (if possible).
		if (p_device->readable()) {
			p_pc->putc(p_device->getc());
		}
	}
}

void IMU::setCalibrationEnabled(bool isEnabled) {
	calibrationEnabled = isEnabled;
	// If we're turning calibration on, clear the variables that stored the old
	// calibration data.
	if (isEnabled) {
		sumAccX = sumAccY = sumAccZ = sumGyrX = sumGyrY = sumGyrZ = sumMagX = sumMagY = sumMagZ = num = 0;
		minGyrX = minGyrY = minGyrZ = INT_MAX;
		maxGyrX = maxGyrY = maxGyrZ = -INT_MAX;
	}
}

void rx_interrupt_imu()
{
	// Indicate that the interrupt was called.
	led2 = !led2;
	
	while (imu.p_device->readable()) {
		imu.buffer[imu.i_buffer_write] = imu.p_device->getc();
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
