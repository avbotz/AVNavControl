
#include "mbed.h"
#include <stdarg.h>

#ifndef DEBUG_H_
#define DEBUG_H_

//attempts to print a string to serial

void inline print_serial(Serial* serial, const char* format, ...) {
	char buffer[256];
	va_list args;
	va_start (args, format);
	vsprintf(buffer, format, args);
	buffer[255] = '\0';
	for (int i = 0; buffer[i] != '\0'; i++){
		while (!serial->writeable());
		serial->putc(buffer[i]);
	}
	va_end(args);
}

extern DigitalOut led1, led2, led3, led4;

#endif