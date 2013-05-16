// PID is a controller used
// to minimize error and make
// the sub go where it is supposed
// to go

#ifndef PID_H
#define PID_H

#include "pid_library.h"
#include "Kalman.h"
#include "motor.h"
#include "defs.h"
#include "pc.h"

extern Kalman pitchK, rollK;

extern volatile float calcP, calcH, calcR;
extern float accP, accR;
extern float accX, accY, accZ, gyrX, gyrY, gyrZ;
extern float imu_data[9];
extern float MU_X_ACC, MU_Y_ACC, MU_Z_ACC, MU_X_GYR, MU_Y_GYR, MU_Z_GYR, MU_X_COM, MU_Y_COM, MU_Z_COM;

extern PID* pitchPID;
extern PID* headingPID;
extern PID* depthPID;

extern volatile int desHead, desDepth, desPower;
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
