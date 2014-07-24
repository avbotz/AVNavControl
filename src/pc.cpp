#include "pc.h"


PC::PC(PinName tx, PinName rx, int baud)
{
	desired_power = 100;
	desired_heading = 0;
	desired_depth = 20;
	p_device = new Serial(tx, rx);
	p_device->baud(baud);
	
	tx_buffer = new CircularBuffer(PC_BUFFER_SIZE);
	rx_buffer = new CircularBuffer(PC_BUFFER_SIZE);
	

/*
 * This is the buffer for the message that will be sent to the BeagleBoard.
 * Format: 'h', AVNav-encoded data for heading,
 *         'd', AVNav-encoded data for depth (pressure sensor),
 *         'p', AVNav-encoded data for power (motors),
 *         kill switch status (l is dead and k is alive),
 *         '\n' to terminate the message.
 */
	mes = new char[12];
	// Copy an initial message to the buffer. The pipe characters (|) will be
	// replaced with AVNav-encoded numbers when this is actually used. We chose
	// the pipe character because it cannot be confused with AVNav-encoded
	// data -- see comments in encode_avnav().
	strcpy(mes, "h||d||p|||\n");
}

// The avnav functions encode data into AVNav format: 
// 
// - 0x00 to 0x1f reserved for low-level commands, like legacy, pre-mbed bootloader
//   commands. (see dsBOOT or Bootloader);
// - 0x20 ASCII space to 0x5f ASCII underscore (_) used for actually encoding
//   data; and
// - all the lowercase letters reserved for commands or control metadata.
// 
// This ensures that if a byte is dropped, it's unambiguous whether the next 
// byte is a command or data. Downside is that the data is less human-readable,
// but it's not like the BeagleBoard is a person or sentient being.

avnav PC::encode_avnav(int data)
{
	avnav encoded;
	
	data &= 0x3fff;
	encoded.byte1 = (data >> 6) + 0x20;
	encoded.byte2 = (data & 0x3f) + 0x20;
	
	return encoded;
}

int PC::decode_avnav(avnav data)
{
	return ((data.byte1 - 0x20) << 6) | (data.byte2 - 0x20);
}

void send_status_pc()
{
	avnav avnav_temp;

	// Construct the message to the computer based on state of the sub.
	// get & encode heading value
	avnav_temp = pc.encode_avnav(calcH);	//back motor
	pc.mes[1] = avnav_temp.byte1;
	pc.mes[2] = avnav_temp.byte2;
	
	// depth (pressure sensor)
	avnav_temp = pc.encode_avnav(depth);	//front motor
	pc.mes[4] = avnav_temp.byte1;
	pc.mes[5] = avnav_temp.byte2;
	
	// power (motor)
	avnav_temp = pc.encode_avnav(desPower);
	pc.mes[7] = avnav_temp.byte1;
	pc.mes[8] = avnav_temp.byte2;
	
	// kill switch
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
	// Loop while the character is not a null terminator
	while (message[i])
	{
		putc(message[i]);
		i++;
	}
}

void PC::putc(char c)
{
	NVIC_DisableIRQ(UART0_IRQn);
	tx_buffer->writeByte(c);
	NVIC_EnableIRQ(UART0_IRQn);
	// Don't worry about overflow because if you're 1024 chars behind you're FUBAR already
}

bool PC::isTxEmpty() const
{
	return tx_buffer->empty;
}

void tx_interrupt_pc()
{
	while (!pc.tx_buffer->empty)
	{
		if (pc.p_device->writeable())
		{
			pc.p_device->putc(pc.tx_buffer->readByte());
		}
	}
	
//	while (pc.p_device->writeable() && !pc.tx_buffer->empty)
//	{
//		pc.p_device->putc(pc.tx_buffer->readByte());
//	}

	if (pc.tx_buffer->empty)
	{
	//	NVIC_DisableIRQ(UART0_IRQn);
		// if nothing to write, turn off the interrupt until motor.getc() is called again
	}
}

void rx_interrupt_pc()
{
	while (pc.p_device->readable())
	{
		NVIC_DisableIRQ(UART0_IRQn);
		pc.rx_buffer->writeByte(pc.p_device->getc());
		NVIC_EnableIRQ(UART0_IRQn);
		if (pc.rx_buffer->overflow)
		{
	//		NVIC_DisableIRQ(UART0_IRQn);
			break;
		}
	}
}


//in debug mode it returns the next character
//otherwise it reads characters and updates the desired stuff
char PC::readPC()
{
	if (debug)
	{
		if (rx_buffer->empty)
		{
			return 0;
		}
		else
		{
			return rx_buffer->readByte();
		}
	}
	
	while (rx_buffer->hasData(4) &&
			!rx_buffer->peek(3) == '\n')
	{
		//throw out the data cuz its bad
		rx_buffer->readByte();
	}
	
	NVIC_DisableIRQ(UART0_IRQn);
	while ((rx_buffer->hasData(4) &&		//make sure there are 4 characters to be read 
			rx_buffer->peek(3) == '\n'))	//and the fourth character is a newline
	{
		
		avnav temp;
		switch (rx_buffer->readByte())
		{
			//read 2 bytes, process them, and set the right variables
			//the last increment skips the newline
		case 'h':
			temp.byte1 = rx_buffer->readByte();
			temp.byte2 = rx_buffer->readByte();
			desired_heading = decode_avnav(temp);
			break;
		case 'd':
			temp.byte1 = rx_buffer->readByte();
			temp.byte2 = rx_buffer->readByte();
			desired_depth = decode_avnav(temp);
			break;
		case 'p':
			temp.byte1 = rx_buffer->readByte();
			temp.byte2 = rx_buffer->readByte();
			desired_power = decode_avnav(temp);
			break;
		}
		rx_buffer->readByte();	//advance past the newline
		
	}
	NVIC_EnableIRQ(UART0_IRQn);
	return 1;
}
