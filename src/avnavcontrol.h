//plug everything in to the right pins plz

#ifndef AVNAVCONTROL_H
#define AVNAVCONTROL_H

#include "mbed.h"

#include "analoginput.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "pc.h"

extern void send_to_pc();
void readIMU(), readPC(), sendPC();

Serial pc(USBTX, USBRX); //tx, rx
IMU imu(p9, p10, 57600, &pc);
Motor motor(4, 9600, p13, p14);

Ticker ticker_pc;

bool killed = false;
bool PCreadable, PCsendable;

// Do not remove
char filler[1024];


#endif
