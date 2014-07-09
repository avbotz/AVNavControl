//plug everything in to the right pins please

#ifndef AVNAVCONTROL_H
#define AVNAVCONTROL_H

#include "mbed.h"
#include "imu.h"

extern void send_to_pc();
void readIMU(), readPC(), sendPC(), sendMotor();

extern PC pc;

extern volatile bool isAlive;

bool PCreadable, PCsendable;

extern float imu_data[9];

extern bool debug;

extern DigitalOut dropperRelay;

#endif
