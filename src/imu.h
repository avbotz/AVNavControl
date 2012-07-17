#ifndef _IMU_H
#define _IMU_H

#include "mbed.h"
#include <limits.h>

#include "debug.h"
#include "pc.h"
#include "defs.h"

void rx_interrupt_imu();

class IMU {
// TODO: Figure out which functions and variables can be private.
public:
	IMU(PinName tx, PinName rx, int baud, PC* pc);
	~IMU();
	
	friend void rx_interrupt_imu();
	
	void putc(char);
	bool readable();
	void parse();
	void getData();
	void directAccess();
	void setCalibrationEnabled(bool isEnabled);
	
	// ticks per degree, from <http://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf>
	static const float gyroScale = 0.069565f;
	static const float radToDeg = 57.29577951308232f; // equals 180/pi
	
	short accX, accY, accZ, 
		  gyrX, gyrY, gyrZ, 
		  magX, magY, magZ;
	unsigned short heading; // 0 to 359. current compass heading set by calcHeading()
	
	bool parseNow;
	
	// Calibration variables
	long long sumAccX, sumAccY, sumAccZ,
	          sumGyrX, sumGyrY, sumGyrZ,
	          sumMagX, sumMagY, sumMagZ;
	int minGyrX, maxGyrX, minGyrY, maxGyrY, minGyrZ, maxGyrZ;
	int num;
	
	Serial* p_device;
	Serial* p_pc;
	
private:
	bool calibrationEnabled;
	
	// Stores a line of raw data from the IMU
	char buffer[IMU_RX_BUFFER_SIZE];
	int i_buffer_read, i_buffer_write;
	char linebuf[1024];
	int i_linebuf;
	bool buffer_overflow;
};

void rx_interrupt();

extern IMU imu;

#endif
