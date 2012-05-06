#ifndef _IMU_H
#define _IMU_H

#include "mbed.h"
#include <string>
#include <sstream>
#include <limits.h>

class IMU {
// TODO: Figure out which functions and variables can be private.
public:
	IMU(PinName tx, PinName rx, int baud, Serial* pc);
	~IMU();
	
	void putc(char);
	char getc();
	bool readable();
	void attach(void (*fptr)(void));
	void parse(char*);
	void getData();
	void directAccess();
	void calcHeading();
	
	// ticks per degree, from <http://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf>
	static const float gyroScale = 0.069565f;
	static const float radToDeg = 57.29577951308232f; // equals 180/pi
	
	// Stores a line of raw data from the IMU
	char buffer[1000];
	int buffer_index;
	int buffer_find(char c);
	
	short accX, accY, accZ, 
		  gyrX, gyrY, gyrZ, 
		  magX, magY, magZ;
	unsigned short heading; // 0 to 359. current compass heading set by calcHeading()
	
	bool IMUreadable;
	bool PCreadable;
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
