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
	void setConstants(double, double, double, double, double, double, double, double, double);
	
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
	
	CircularBuffer* rx_buffer;
	// Stores a line of raw data from the IMU
	char linebuf[1024];
	int i_linebuf;

	// Average values with IMU on flat surface.
	double MU_X_ACC = 27.888451;
	double MU_Y_ACC = 30.980315;
	double MU_Z_ACC = (245.883202 - 256.0f);
	// Subtract 256 in the Z direction to account for gravity. According to IMU data
	// sheet, the sensitivity at 2g mode (which is what we are using) is
	// 256 least significant bits per 1 g. Normal Earth gravity = 1 g.

	double MU_X_GYR = -58.106299;
	double MU_Y_GYR = 17.059055;
	double MU_Z_GYR = 11.618110;

	double MU_X_COM = -58.242588;
	double MU_Y_COM = -114.202156;
	double MU_Z_COM = -25.672507;
};

void rx_interrupt();

extern IMU imu;

#endif
