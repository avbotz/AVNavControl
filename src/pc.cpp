#include "pc.h"


PC::PC(PinName tx, PinName rx, int baud)
{
	desired_power = 100;
	desired_heading = 0;
	desired_depth = 20;
	p_device = new Serial(tx, rx);
	p_device->baud(baud);

	// Set an interrupt to send data to the BeagleBoard every 1.0 seconds.

	for (int i = 0; i < PC_BUFFER_SIZE; i++) {
		rx_buffer[i] = tx_buffer[i] = 0;
	}

	i_tx_read = i_tx_write = 0;
	i_rx_read = i_rx_write = 0;

	tx_empty = true;


	/*
 * This is the buffer for the message that will be sent to the BeagleBoard.
 * Format: 'h', AVNav-encoded data for heading,
 *         'd', AVNav-encoded data for depth (pressure sensor),
 *         'p', AVNav-encoded data for power (motors),
 *         kill switch status (l is dead and k is alive),
 *         '\n' to terminate the message.
 */
	mes = new char[12];
	strcpy(mes, "h||d||p|||\n");
}

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

avnav PC::encode_avnav(int data) {
	avnav encoded;
	
	data &= 0x3fff;
	encoded.byte1 = (data >> 6) + 0x20;
	encoded.byte2 = (data & 0x3f) + 0x20;
	
	return encoded;
}

int PC::decode_avnav(avnav data) {
	return ((data.byte1 - 0x20) << 6) | (data.byte2 - 0x20);
}

extern float ppower;

void send_status_pc()
{
	depth = pressure.getValueCalibrated();
	avnav avnav_temp;

	// Construct the message to the BeagleBoard based on state of the sub.
	// get heading value
	avnav_temp = pc.encode_avnav(calcH);	//back motor
	pc.mes[1] = avnav_temp.byte1;
	pc.mes[2] = avnav_temp.byte2;
	
	// get depth (pressure sensor) value
	avnav_temp = pc.encode_avnav(depth);	//front motor
	pc.mes[4] = avnav_temp.byte1;
	pc.mes[5] = avnav_temp.byte2;
	
	// get power (motor) value
	avnav_temp = pc.encode_avnav(desPower);
	pc.mes[7] = avnav_temp.byte1;
	pc.mes[8] = avnav_temp.byte2;
	
	// get kill switch value
	// TODO: is this correct? I assume voltage on kill pin means alive.
	pc.mes[9] = isAlive ? 'k' : 'l';
	/*
				char* tmp;
			sprintf(tmp, "%d", pressure.getValueCalibrated());
			pc.send_message(tmp);
			delete tmp;*/
	// print message to PC
	pc.send_message(pc.mes);
}

void PC::send_message(const char* message)
{
	int i = 0;
	while (message[i]) {
		putc(message[i]);
		i++;
	}
}

void PC::putc(char c)
{
	tx_buffer[i_tx_write] = c;
	NVIC_DisableIRQ(UART1_IRQn);
	i_tx_write = (i_tx_write + 1) % PC_BUFFER_SIZE;
	NVIC_EnableIRQ(UART1_IRQn);
	// Don't worry about overflow because if you're 1024 chars behind you're FUBAR already
}

void tx_interrupt_pc()
{
	while (pc.p_device->writeable() && pc.i_tx_write != pc.i_tx_read) {
		pc.p_device->putc(pc.tx_buffer[pc.i_tx_read]);
		pc.i_tx_read = (pc.i_tx_read + 1) % PC_BUFFER_SIZE;
		pc.tx_empty = false;
	}
	if (pc.i_tx_write == pc.i_tx_read) {
		pc.tx_empty = true;
		NVIC_DisableIRQ(UART1_IRQn);
		// if nothing to write, turn off the interrupt until motor.getc() is called again
	}
}

void rx_interrupt_pc()
{
	while (pc.p_device->readable()) {
		pc.rx_buffer[pc.i_rx_write] = pc.p_device->getc();
		NVIC_DisableIRQ(UART0_IRQn);
		pc.i_rx_write = (pc.i_rx_write + 1) % PC_BUFFER_SIZE;
		NVIC_EnableIRQ(UART0_IRQn);
		if (pc.i_rx_write == pc.i_rx_read) {
			pc.rx_overflow = true;
			NVIC_DisableIRQ(UART0_IRQn);
			break;
		}
	}
}


//in debug mode it returns the next character
//otherwise it reads characters and updates the desired stuff
char PC::readPC()
{
	if (debug) {
		if (i_rx_read == i_rx_write) return 0;
		char ret = rx_buffer[i_rx_read];
		i_rx_read = (i_rx_read + 1) % PC_BUFFER_SIZE;
		return ret;
	}
	
	while ((i_rx_read != i_rx_write &&
			((i_rx_read + 1) % PC_BUFFER_SIZE) != i_rx_write &&		//make sure there are 4 characters to be read
			((i_rx_read + 2) % PC_BUFFER_SIZE) != i_rx_write &&
			((i_rx_read + 3) % PC_BUFFER_SIZE) != i_rx_write &&
			(rx_buffer[(i_rx_read + 3) % PC_BUFFER_SIZE] == '\n')) || //and the fourth character is a newline
			rx_overflow) { 
		rx_overflow = false;
		avnav temp;
		switch (rx_buffer[i_rx_read]) {
			//read 2 bytes, process them, and set the right variables
			//the last increment skips the newline
		case 'h':
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			temp.byte1 = rx_buffer[i_rx_read];
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			temp.byte2 = rx_buffer[i_rx_read];
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			desired_heading = decode_avnav(temp);
			break;
		case 'd':
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			temp.byte1 = rx_buffer[i_rx_read];
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			temp.byte2 = rx_buffer[i_rx_read];
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			desired_depth = decode_avnav(temp);
			break;
		case 'p':
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			temp.byte1 = rx_buffer[i_rx_read];
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			temp.byte2 = rx_buffer[i_rx_read];
			i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
			desired_power = decode_avnav(temp);
			break;
		}

		i_rx_read = (i_rx_read+1)%PC_BUFFER_SIZE;
	}
	return 1;
}
