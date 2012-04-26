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
	parseNow = PCreadable = IMUreadable = false;
	
	debugMode = false;
	
	for (int i = 0; i < 75; i++) {
		buffer[i] = 0;
	}
		
	p_device->putc('4');	// tell the imu to start sending in case it isn't doing that already.
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

int IMU::buffer_find(char c) {
	for (int i = 0; i < buffer_index; i++) {
		if (buffer[i] == c) return i;
	}
	return buffer_index;
}


// Checks data integrity of buffer, then stores the IMU values into variables.
void IMU::parse(char* buf) {
	/*if (buffer_find('$') != buffer_index && buffer_find('#') != buffer_index) {
		// Replace '$' ',' '#' with space ' ' so that we can use stringstream
		for (int i = 0; i < buffer_index; i++) {
			if (buf[i] == '$' || buf[i] == ',' || buf[i] == '#') {
				buf[i] = ' ';
			}
		} */
		
		// Parse out the numbers with sscanf
		// TODO: sscanf returns stuff based on the parse's success. should we check for that?
	p_device->scanf(buf, "$%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd#", &accX, &accY, &accZ, &gyrX, &gyrY, &gyrZ, &magX, &magY, &magZ);
	//p_pc->printf("%s\r\n", buf);
		//stringstream ss(*buf);
		//ss >> accX >> accY >> accZ >> gyrX >> gyrY >> gyrZ >> magX >> magY >> magZ;
		//p_pc->printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n\r", accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ);
		char printtemp[75];
		sprintf(printtemp, "%d, %d, %d, %d, %d, %d, %d, %d, %d\n\r", accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ);
		for (int i = 0; i < sizeof(printtemp)/sizeof(printtemp[0]); i++) {
			p_pc->putc(printtemp[i]);
		} 
				//calcHeading();
	//}
	//else {
	//	p_pc->printf("Bad parse!\n\r");
	//}
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
	p_device->attach(fptr);
}

void IMU::getData() {
	//p_pc->printf("hi");
	
	if (IMUreadable) {
	   // p_pc->printf("imu readable\n\r");
	   
		if (debugMode) {
			while (readable()) {
				
				char c = getc();
				//p_pc->putc(c);
				//if (c == '#') p_pc->printf("where the hell is my pound?\r\n");
				
			}
		}
		else {
			while (readable()) {

				char c = getc();
				p_pc->putc(c);
				//if (c == '#')
					//p_pc->printf("where the hell is my pound?\r\n");
				//p_pc->printf("pushing\n\r");
				buffer[buffer_index] = c;
				buffer_index++;
				//if (buffer_index == 75) buffer_index = 0;
				//p_pc->printf("pushed char %d %c\n\r", buffer_index , c);
				if (buffer[buffer_index - 1] == '#') {
					parseNow = true;
					break;
				}
				// p_pc->printf("__%d__", parseNow);
			}
		}

		IMUreadable = false;
	}
	
	if (parseNow) {
		//p_pc->printf("calling parse\n\r");
		parse(buffer);
		//p_pc->printf("done parse\n\r");
		
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
		
		// Clear the buffer.
		for (int i = 0; i < buffer_index; i++) {
			buffer[i] = 0;
		}
		buffer_index = 0;
		
		//p_pc->printf("cleared\n\r");
		parseNow = false;
		
	}
		
	if (!NVIC_GetActive(UART3_IRQn)) {
		NVIC_EnableIRQ(UART3_IRQn);
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
	p_pc->printf("calculated compass heading: %d\n\r", heading);
	
	
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
