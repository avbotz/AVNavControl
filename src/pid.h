// PID is a controller used
// to minimize error and make
// the sub go where it is supposed
// to go

#ifndef PID_H
#define PID_H

#include <cmath>

#include "pid_library.h"
#include "Kalman.h"

#ifndef NULL
#define NULL 0
#endif

//PID tuning constants change to tune
//the first list is the most used ones
//change them to tune, the second list
//are the ones used by the mbed, they
//are different and not intuitive

#define PITCH_KP 0.05f
#define PITCH_KI 0.0f
#define PITCH_KD 0.0f

#define HEADING_KP 1.0f
#define HEADING_KI 0.2f
#define HEADING_KD 0.0f

#define DEPTH_KP 0.5f
#define DEPTH_KI 0.0f
#define DEPTH_KD 0.0f

#define P_KC PITCH_KP
#define P_TI (PITCH_KP/PITCH_KI)
#define P_TD (PITCH_KD/PITCH_KP)

#define H_KC HEADING_KP
#define H_TI (HEADING_KP/HEADING_KI)
#define H_TD (HEADING_KD/HEADING_KP)

#define D_KC DEPTH_KP
#define D_TI (DEPTH_KP/DEPTH_KI)
#define D_TD (DEPTH_KD/DEPTH_KP)


//ifndef used because it is defined in
//two seperate places and im not too sure
//who includes who
#ifndef LEFT
#define LEFT 0
#define RIGHT 1
#define FRONT 2
#define BACK 3
#endif

extern Kalman pitchK, rollK;

extern float calcP, calcH, calcR;
extern float accP, accR;
extern float accX, accY, accZ, gyrX, gyrY, gyrZ;

extern PID* pitchPID;
extern PID* headingPID;
extern PID* depthPID;

extern int desHead, desDepth, desPower;
extern float depth;

extern unsigned char motorArray[4];

void reset_pid();
void init_pid();
void do_pid();
void get_compass();
void give_data(int, int, int, int, int, int);
void update_motors(float, float, float);

#endif
