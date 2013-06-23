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

// Page that maps pins to UART numbers.
// http://mbed.org/users/mbed714/notebook/uart-mapping/

//avnavcontrol.h
// USB = UART 0
PC pc(USBTX, USBRX, 115200); //tx, rx

//IMU.h
// pins 9, 10 = UART 3
IMU imu(p9, p10, 57600, &pc);

//motor.h
// pins 13, 14 = UART 1
Motor motor(4, 9600, p13, p14);

DigitalOut led1(LED1), // Kill status. On means alive.
           led2(LED2), // IMU: toggled when a character is received.
           led3(LED3), // IMU: toggled when a buffer is parsed.
           led4(LED4); // IMU: toggled when a character is copied from buffer to linebuf.

bool debug = false;
bool autoCalibrateIMU = false;

volatile bool isAlive = false;
volatile int depth(0);

// Average values with IMU on flat surface.
double MU_X_ACC = 27.888451;
double MU_Y_ACC = 30.980315;
double MU_Z_ACC = (245.883202 - 256.0f);
// Subtract 256 in the Z direction to account for gravity. According to IMU data
// sheet, the sensitivity at 2g mode (which is what we are using) is
// 256 least significant bits per 1 g. Normal Earth gravity = 1 g.

double MU_X_GYR = -58.106299;
double MU_Y_GYR = 17.059055;
double MU_Z_GYR = 11.618110;

double MU_X_COM = -58.242588;
double MU_Y_COM = -114.202156;
double MU_Z_COM = -25.672507;
