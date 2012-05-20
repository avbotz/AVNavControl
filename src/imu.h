#ifndef _IMU_H
#define _IMU_H

#include "mbed.h"
#include <string>
#include <sstream>
#include <limits.h>
#include "debug.h"

#define IMU_RX_BUFFER_SIZE 1024

class IMU {
// TODO: Figure out which functions and variables can be private.
public:
	IMU(PinName tx, PinName rx, int baud, Serial* pc);
	~IMU();
	
	void putc(char);
	char getc();
	bool readable();
	void attach(void (*fptr)(void));
	void parse();
	void getData();
	void directAccess();
	void calcHeading();
	
	// ticks per degree, from <http://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf>
	static const float gyroScale = 0.069565f;
	static const float radToDeg = 57.29577951308232f; // equals 180/pi
	
	// Stores a line of raw data from the IMU
	char buffer[IMU_RX_BUFFER_SIZE];
	int i_buffer_read, i_buffer_write;
	char linebuf[1024];
	int i_linebuf;
	bool buffer_overflow;
	
	short accX, accY, accZ, 
		  gyrX, gyrY, gyrZ, 
		  magX, magY, magZ;
	unsigned short heading; // 0 to 359. current compass heading set by calcHeading()
	
	bool parseNow;
	bool debugMode;
	
	Serial* p_device;
	Serial* p_pc;
	
private:
	// Calibration variables
	bool calibrationEnabled;
	long long sumAccX, sumAccY, sumAccZ,
			  sumGyrX, sumGyrY, sumGyrZ,
			  sumMagX, sumMagY, sumMagZ;
	int minGyrX, maxGyrX, minGyrY, maxGyrY, minGyrZ, maxGyrZ;
	int num;
};

extern IMU imu;

#endif
