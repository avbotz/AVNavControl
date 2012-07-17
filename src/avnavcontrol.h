//plug everything in to the right pins please

#ifndef AVNAVCONTROL_H
#define AVNAVCONTROL_H

#include "mbed.h"
#include "imu.h"

extern void send_to_pc();
void readIMU(), readPC(), sendPC(), sendMotor();

extern PC pc;

volatile bool isAlive = false;
bool PCreadable, PCsendable;

extern bool debug;

#endif
