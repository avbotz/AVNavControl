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
#define FRONT 1
#define LEFT 2
#define BACK 3


//pc.h
#define PC_BUFFER_SIZE 1024


//pid.h

#define DEAD_TIME 5000

//PID tuning constants change to tune
//the first list is the most used ones
//change them to tune, the second list
//are the ones used by the mbed, they
//are different and not intuitive

#define PITCH_KP 0.012f
#define PITCH_KI 0.005f
#define PITCH_KD 0.007f

#define HEADING_KP 4.0f
#define HEADING_KI 0.75f
#define HEADING_KD 0.25f

#define DEPTH_KP 1.0f
#define DEPTH_KI 0.1f
#define DEPTH_KD 0.2f

// motor multipliers
#define RIGHT_MULTIPLIER .848 // right is stronger
#define FRONT_MULTIPLIER 1
#define LEFT_MULTIPLIER 1
#define BACK_MULTIPLIER -.8 // back is backwards and stronger

#endif
