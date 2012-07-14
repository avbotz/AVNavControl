// PID is a controller used
// to minimize error and make
// the sub go where it is supposed
// to go

#ifndef PID_H
#define PID_H

#include "pid_library.h"
#include "Kalman.h"
#include "motor.h"

#ifndef NULL
#define NULL 0
#endif

//PID tuning constants change to tune
//the first list is the most used ones
//change them to tune, the second list
//are the ones used by the mbed, they
//are different and not intuitive

#define PITCH_KP 0.01f
#define PITCH_KI 0.0f
#define PITCH_KD 0.0f

#define HEADING_KP 1.0f
#define HEADING_KI 0.2f
#define HEADING_KD 0.0f

#define DEPTH_KP 0.8f
#define DEPTH_KI 0.05f
#define DEPTH_KD 0.0f

extern Kalman pitchK, rollK;

extern float calcP, calcH, calcR;
extern float accP, accR;
extern float accX, accY, accZ, gyrX, gyrY, gyrZ;

extern PID* pitchPID;
extern PID* headingPID;
extern PID* depthPID;

extern int desHead, desDepth, desPower;
extern volatile int depth;
extern bool debug;

extern volatile bool isAlive;

extern unsigned char motorArray[4];
extern AnalogKillSwitch kill;

void reset_pid();
void init_pid();
void do_pid();
void get_compass();
void give_data(int, int, int, int, int, int);
void update_motors(float, float, float);

#endif
