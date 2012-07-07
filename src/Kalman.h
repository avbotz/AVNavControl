#ifndef Kalman_h
#define Kalman_h

#include "mbed.h"

#define GYR_SCALE	1/14.375f  //degrees per LSB
#define SAMPLES_PER_SECOND 70  //measured in Hz
#define DT 1.0f/SAMPLES_PER_SECOND


 // Average values with IMU on flat surface.
#define MU_X_ACC -46.702186
#define MU_Y_ACC 28.008197
#define MU_Z_ACC (243.980874 - 256.0f)
// Subtract 256 in the Z direction to account for gravity. According to IMU data
// sheet, the sensitivity at 2g mode (which is what we are using) is
// 256 least significant bits per 1 g. Normal Earth gravity = 1 g.

#define MU_X_GYR -47.894809
#define MU_Y_GYR 24.949454
#define MU_Z_GYR 2.669399

#define MU_X_COM -121.478142
#define MU_Y_COM -120.972678
#define MU_Z_COM 22.863388

/*
 * Kalman filters are a "sensor fusion" filter, combining orientation data from
 * multiple sensors and taking into account the past orientation and the dynamics
 * of the submarine.
 */

class Kalman {
public:
	Kalman(float g_bias);
	~Kalman();

	float calculate(int gyroReading, float accTheta);
	
private:
	// Kalman filter data
	float
	angle,      // data
	bias,       // gyroscope bias
	angle_err,  // angle error
	P[2][2],    // covariance
	E,          // error estimate
	gain[2];    // Kalman filter gains
	
	//float dAngle, dBias, dAngle_err, dP[2][2], dE, dGain[2];

	/*
	 * Q is a 2x2 matrix of the covariance of the process noise. Because we
	 * assume the gyro and accelerometer noise to be independent
	 * of each other, the covariances on the / diagonal are 0.
	 *
	 * Covariance Q, the process noise, from the assumption
	 *	x = F x + B u + w
	 * with w having a normal distribution with covariance Q.
	 * (covariance = E[ (X - E[X])*(X - E[X])' ]
	 * We assume it is linear with dt
	 */
	static const float Q_angle = 0.05f;
	static const float Q_gyro = 0.15f;
	
	/*
	 * R represents the measurement covariance noise.  In this case,
	 * it is a 1x1 matrix that says that we expect 17.2 degrees of jitter
	 * from the accelerometer. Both R and Q need to be updated for the new IMU.
	 * See http://en.wikipedia.org/wiki/Kalman_filter#Estimation_of_the_noise_covariances_Qk_and_Rk
	 */
	static const float R_angle = 25.0f;


	static const float SCALE = 1/14.375f;
	//probably needs to be changed for new IMU. maybe pass in via constructor?
	//I'm pretty sure this is the same thing as GYRO_SCALE in main.cpp, although the numbers
	//are surprisingly different

};

#endif
