#include "pc.h"

/*avnav encode_avnav(int data) {
	avnav encoded;
	return encoded;
}

// TODO: explain these shenanigans
int decode_avnav(avnav data){
	return ((data.byte1 - 0x20) << 6) | (data.byte2 - 0x20);
}*/

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

