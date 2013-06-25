#ifndef DEFS_H
#define DEFS_H

//This header is intended to put all #defines in a single file.
//This will hopefully prevent clutter.

//imu.h
#define IMU_RX_BUFFER_SIZE 1024


//Kalman.h
#define GYRO_SCALE	1/14.375f  //degrees per LSB
#define SAMPLES_PER_SECOND 70  //measured in Hz
#define DT 1.0f/SAMPLES_PER_SECOND

//motors.h
#define SYNC_BYTE 255
#define MOTOR_TX_BUF_SIZE 1024

#define RIGHT 0
#define LEFT 1
#define FRONT 2
#define BACK 3


//pc.h
#define PC_BUFFER_SIZE 1024


//pid.h

//PID tuning constants change to tune
//the first list is the most used ones
//change them to tune, the second list
//are the ones used by the mbed, they
//are different and not intuitive

#define PITCH_KP 0.001f
#define PITCH_KI 0.0f
#define PITCH_KD 0.0f

#define HEADING_KP 2.5f
#define HEADING_KI 0.75f
#define HEADING_KD 0.25f

#define DEPTH_KP 1.0f
#define DEPTH_KI 0.1f
#define DEPTH_KD 0.2f

#endif