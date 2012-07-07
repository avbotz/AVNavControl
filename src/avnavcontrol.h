//plug everything in to the right pins plz

#ifndef AVNAVCONTROL_H
#define AVNAVCONTROL_H

#include "mbed.h"
#include "imu.h"

extern void send_to_pc();
void readIMU(), readPC(), sendPC(), sendMotor();

extern PC pc;

bool isAlive = false;
bool PCreadable, PCsendable;

extern bool debug;


// Do not remove
char filler[1024];

#endif
