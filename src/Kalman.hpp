#ifndef Kalman_h
#define Kalman_h

#include "mbed.h"
#include "defs.h"

/*
 * Kalman filters are a "sensor fusion" filter, combining orientation data from
 * multiple sensors and taking into account the past orientation and the dynamics
 * of the submarine.
 */

class Kalman
{
public:
	Kalman();
	~Kalman();

	float calculate(int gyroReading, float accTheta);
	void setBias(float g_bias);

private:
	// Kalman filter data
	float
	angle,      // data
	bias,       // gyroscope bias
	angle_err,  // angle error
	P[2][2],    // covariance
	E,          // error estimate
	gain[2];    // Kalman filter gains

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

};

#endif
