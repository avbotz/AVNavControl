/*
 * IMPORTANT
 * The goal of this file is to provide a place to initialize all of the variables that
 * could cause problems due to multiple definitions if they are declared in header files
 */


#include "analog.h"
#include "imu.h"
#include "motor.h"
#include "pc.h"

//analoginput.h
// TODO: check the magic threshold 0.85f. Too high/low?
AnalogKillSwitch kill(p16, p18, 0.85f); // Vin, Vout

/* 
 * Daniel's was depth = adc_buffer[ADC_PRESS] * 0.361904762 - 101.33333;
 * That was based on the dsPIC33FJ family's ADC converter.
 * Also he used linear regression to convert depth sensor voltage to feet
 * Adding 0.5 so that it rounds correctly.
 * The resistor that we're using on the pressure sensor is 2.2K ohms.
 */
AnalogPressureSensor pressure(p17, 341.74, -48.107);

// UART mapping http://mbed.org/users/mbed714/notebook/uart-mapping/

//avnavcontrol.h
// USB = UART 0
PC pc(USBTX, USBRX, 115200); //tx, rx

//IMU.h
// pins 9, 10 = UART 3
IMU imu(p9, p10, 57600, &pc);

//motor.h
// pins 13, 14 = UART 1
Motor motor(5, 9600, p13, p14);

DigitalOut led1(LED1),	// Kill status
           led2(LED2),	// debug
           led3(LED3),	// debug
           led4(LED4);	// debug

bool debug = false;

//dropPositions[0] = value mbed should send the Pololu when nothing is to be dropped.
//dropPositions[1] = value mbed should send the Pololu when one markers is to be dropped.
//dropPositions[2] = extrapolate
unsigned char dropPositions[3] = {0, 127, 254};

