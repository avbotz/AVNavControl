#ifndef _IMU_H
#define _IMU_H

#include "mbed.h"

#include "debug.h"
#include "pc.h"
#include "defs.h"

void rx_interrupt_imu();

class IMU
{
// TODO: Figure out which functions and variables can be private.
public:
	IMU(PinName tx, PinName rx, int baud, PC* pc);
	~IMU();
	
	// TODO: no longer necessary; interrupts can bind to members. See mbed API.
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
	int num;
	
	Serial* p_device;
	Serial* p_pc;
	
private:
	bool calibrationEnabled;
	
	CircularBuffer* rx_buffer;
	// Stores a line of raw text from the IMU
	char linebuf[1024];
	int i_linebuf;
};

void rx_interrupt();

extern IMU imu;

#endif
