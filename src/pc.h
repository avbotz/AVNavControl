#ifndef PC_H
#define PC_H

#include <mbed.h>

#include "analog.h"
#include "pid.h"

struct avnav {
	char byte1;
	char byte2;
};

avnav encode_avnav(int data);
int decode_avnav(avnav data);
void send_to_pc();

// This is the buffer for the message that will be sent to the BeagleBoard.
// Format: 'h', AVNav-encoded data for heading,
//         'd', AVNav-encoded data for depth (pressure sensor),
//         'p', AVNav-encoded data for power (motors),
//         kill switch status (l is dead and k is alive),
//         '\n' to terminate the message.
char mes[] = "h||d||p|||\n";
#define MESSAGE_LENGTH (sizeof(mes)/sizeof(char))


extern Serial pc;

#endif
