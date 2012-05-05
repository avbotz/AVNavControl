
#include "mbed.h"

#ifndef DEBUG_H_
#define DEBUG_H_

//attempts to print a string to serial

void inline print_serial(Serial* port, const char* str) {
	for (int i = 0; str[i] != '\0'; i++) {
		while (!port->writeable());
		port->putc(str[i]);
	}
}



#endif