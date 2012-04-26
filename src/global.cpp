//IMPORTANT
//The goal of this file is to provide a place to initialize all of the variables that
//could cause problems due to multiple definitions if they are declared in header files


#include "analoginput.h"
#include "avnavcontrol.h"
#include "IMU.h"
#include "motor.h"

//analoginput.h
AnalogInput kill(p16, 0.5f);
AnalogInput pressure(p17, 0.5f);


//avnavcontrol.h
Serial pc(USBTX, USBRX); //tx, rx

//IMU.h
IMU imu(p9, p10, 57600, &pc);

//motor.h
Motor motor(4, 9600, p13, p14);