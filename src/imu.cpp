#include "mbed.h"
#include "imu.h"

IMU::IMU(PinName tx, PinName rx, int baud, Serial* pc) {
	p_pc = pc;
	p_device = new Serial(tx, rx);
	p_device->baud(baud);
	p_device->format(8, Serial::None, 1);   // serial settings for the IMU--refer to IMU's data sheet
	
	// Calibration variables
	calibrationEnabled = false;
	if (calibrationEnabled) {
		sumAccX = sumAccY = sumAccZ = sumGyrX = sumGyrY = sumGyrZ = sumMagX = sumMagY = sumMagZ = num = 0;
		minGyrX = minGyrY = minGyrZ = INT_MAX;
		maxGyrX = maxGyrY = maxGyrZ = -INT_MAX;
	}
	
	accX = accY = accZ = gyrX = gyrY = gyrZ = magX = magY = magZ = 0;
	parseNow = false;
	
	debugMode = false;
	
	for (int i = 0; i < IMU_RX_BUFFER_SIZE; i++) {
		buffer[i] = 0;
	}
	i_buffer_write = i_buffer_read = 0;
	
	for (int i = 0; i < 1024; i++) {
		linebuf[i] = 0;
	}
	i_linebuf = 0;
	buffer_overflow = false;
	
	//p_device->putc('4');	// tell the imu to start sending in case it isn't doing that already.
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
	// we <3 pointer arithmetic
	if (sscanf(linebuf, "\n\r$%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd#", temp, temp+1, temp+2, temp+3, temp+4, temp+5, temp+6, temp+7, temp+8) == 9){
		accX = temp[0];
		accY = temp[1];
		accZ = temp[2];
		gyrX = temp[3];
		gyrY = temp[4];
		gyrZ = temp[5];
		magX = temp[6];
		magY = temp[7];
		magZ = temp[8];
		
		led3 = !led3;
	}

	//char printtemp[75];
	//sprintf(printtemp, "%d, %d, %d, %d, %d, %d, %d, %d, %d\n\r", accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ);
	//print_serial(p_pc, printtemp);
}


bool IMU::readable() {
	return p_device->readable();
}

char IMU::getc() {
	return p_device->getc();
}

/* 
 * Wrapper function to attach a callback function to the RX interrupt of the 
 * IMU Serial object. The function is called whenever the IMU sends a 
 * character.
 */
void IMU::attach(void (*fptr)(void)) {
	//p_pc->putc('a');
	// Attach it as an RX interrupt only.
	p_device->attach(fptr, Serial::RxIrq);
}

void IMU::getData() {
	//print_serial(p_pc, "hello");
	//p_pc->printf("hi");
	
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
			
			p_pc->printf(
				"Averages: %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
				sumAccX/(double)num, sumAccY/(double)num, sumAccZ/(double)num,
				sumGyrX/(double)num, sumGyrY/(double)num, sumGyrZ/(double)num,
				sumMagX/(double)num, sumMagY/(double)num, sumMagZ/(double)num
			);
			
		}
		
		// Clear the line buffer.
		for (int i = 0; i < 1024; i++) {
			linebuf[i] = 0;
		}
		i_linebuf = 0;
		
		//p_pc->printf("cleared\n\r");
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

void IMU::calcHeading() {
	// returns arctan(y/x) in radians between -pi and pi
	float h = atan2((float)magY, (float)magX);
	h = h * radToDeg; // convert to degrees. Now h is between -180 and 180.
	
	// retarded code
	//h += 180; // now it's between 0 and 360.
	
	// Convert into degrees heading (i.e., 0 deg means we're going forward)
	h = 90 - h;
	
	// for 2nd quadrant --- is this necessary?
	while (h < 0) {
		h += 360;
	}
	
	heading = (short)(h + 0.5f); // adding 0.5 so that it rounds instead of truncating.
	//p_pc->printf("calculated compass heading: %d\n\r", heading);
	
	
	/*
	//Adit's implementation
	
	float h = atan2((float)magX, (float)magY);
	h *= radToDeg;
	
	// if arctan gave the correct result then only worry about negative angles
	// otherwise add 180 (i think?) to transform the angle to where it should be (on the opposite side since magY is negative)
	if (maxY > 0) {
		h += (h < 0) ? 360 : 0;
	}
	else {
		h += 180;
	}
	*/
	
		
}
