#include "mbed.h"
#include <stdarg.h>

#ifndef DEBUG_H_
#define DEBUG_H_


// Prints to a serial port. Max string size is 255.
void inline print_serial(Serial* serial, const char* format, ...)
{
	char buffer[256];
	va_list args;
	va_start (args, format);
	vsprintf(buffer, format, args);
	buffer[255] = '\0';	//Ensure that the buffer is null terminated.
	
	// We print one character at a time because we drop characters otherwise.
	for (int i = 0; buffer[i] != '\0'; i++)
	{
		while (!serial->writeable());
		serial->putc(buffer[i]);
	}
	va_end(args);
}

//Lights!
extern DigitalOut led1, led2, led3, led4;

#endif
