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


// Average values with IMU on flat surface.
#define MU_X_ACC 26.377091
#define MU_Y_ACC 41.562420
#define MU_Z_ACC (244.498069 - 256.0f)
// Subtract 256 in the Z direction to account for gravity. According to IMU data
// sheet, the sensitivity at 2g mode (which is what we are using) is
// 256 least significant bits per 1 g. Normal Earth gravity = 1 g.

#define MU_X_GYR -63.737452
#define MU_Y_GYR 13.231660
#define MU_Z_GYR 11.486486

#define MU_X_COM -58.242588
#define MU_Y_COM -114.202156
#define MU_Z_COM -25.672507


//motors.h
#define SYNC_BYTE 255
#define MOTOR_TX_BUF_SIZE 1024

#define LEFT 0
#define RIGHT 1
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

#define HEADING_KP 1.0f
#define HEADING_KI 0.2f
#define HEADING_KD 0.0f

#define DEPTH_KP 0.9f
#define DEPTH_KI 0.2f
#define DEPTH_KD 0.0f

#endif