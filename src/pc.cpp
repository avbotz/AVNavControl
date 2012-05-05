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

// TODO: this function is UNTESTED
void send_to_pc() {
	// Holds data while it's being moved around.
	avnav avnav_temp;
	
	// Construct the message to the BeagleBoard based on state of the sub.
	// get heading value
	avnav_temp = encode_avnav(calcH);
	mes[1] = avnav_temp.byte1;
	mes[2] = avnav_temp.byte2;
	
	// get depth (pressure sensor) value
	avnav_temp = encode_avnav(pressure.getValueCalibrated());
	mes[4] = avnav_temp.byte1;
	mes[5] = avnav_temp.byte2;
	
	// get power (motor) value
	avnav_temp = encode_avnav(desPower);
	mes[7] = avnav_temp.byte1;
	mes[8] = avnav_temp.byte2;
	
	// get kill switch value
	// TODO: is this correct? I assume voltage on kill pin means alive.
	mes[9] = kill.getValueThresh() ? 'k' : 'l';
	
	// print message to PC
	if (pc.writeable()) {
		for (int i = 0; i < MESSAGE_LENGTH - 1; i++) {
			pc.putc(mes[i]);
		}
	}
}
