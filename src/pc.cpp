#include "pc.h"


// The avnav functions encode data into AVNav format: 
// 
// - 0x00 to 0x1f reserved for low-level commands, like legacy, pre-mbed bootloader
//   commands. (see dsBOOT or Bootloader);
// - 0x20 ASCII space to 0x5f ASCII underscore (_) used for actually encoding
//   data; and
// - all the lowercase letters reserved for commands or control metadata.
// This ensures that if a byte is dropped, it's unambiguous whether the next 
// byte is a command or data. Downside is that the data is less human-readable,
// but it's not like the BeagleBoard is a person or sentient being.

avnav encode_avnav(int data) {
	avnav encoded;
	
	data &= 0x3fff;
	encoded.byte1 = (data >> 6) + 0x20;
	encoded.byte2 = (data & 0x3f) + 0x20;
	
	return encoded;
}

int decode_avnav(avnav data) {
	return ((data.byte1 - 0x20) << 6) | (data.byte2 - 0x20);
}

void send_to_pc() {
	// This is the buffer for the message that will be sent to the BeagleBoard.
	// Format: 'h', AVNav-encoded data for heading,
	//         'd', AVNav-encoded data for depth,
	//         'p', AVNav-encoded data for pitch,
	//         kill switch status (l is alive and k is dead),
	//         '\n' to terminate the message.
	// TODO: Make sure L means alive and K means dead.
	char mes[] = "h__d__p___\n";
	
	// TODO:
	// get heading
	// get depth
	// get pressure
	// get kill
	// print message to PC
}

