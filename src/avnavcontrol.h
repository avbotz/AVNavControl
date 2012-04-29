//plug everything in to the right pins plz

#ifndef AVNAVCONTROL_H
#define AVNAVCONTROL_H

#include "mbed.h"

#include "analoginput.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"

extern void send_to_pc();
void readIMU(), readPC(), sendPC();

extern Serial pc;

Ticker ticker_pc;

bool killed = false;
bool PCreadable, PCsendable;

DigitalOut led1(LED1),	// Kill status
           led2(LED2),	// 
           led3(LED3),	// 
           led4(LED4);	// 

// Do not remove
char filler[1024];

#endif
