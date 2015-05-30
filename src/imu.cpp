#include "mbed.h"
#include "imu.h"

IMU::IMU(PinName tx, PinName rx, int baud, PC* pc)
{
	// Save a pointer to the PC for debugging use.
	p_pc = pc->p_device;
	p_device = new Serial(tx, rx);
	p_device->baud(baud);
	// serial settings for the IMU--refer to IMU's data sheet
	p_device->format(8, Serial::None, 1);
	
	// Calibration variables
	calibrationEnabled = false;

	accX = accY = accZ = gyrX = gyrY = gyrZ = magX = magY = magZ = 0;
	parseNow = false;
	
	//intialize the buffers
	rx_buffer = new CircularBuffer(IMU_RX_BUFFER_SIZE);
	
	for (int i = 0; i < 1024; i++)
	{
		linebuf[i] = 0;
	}
	i_linebuf = 0;
	
	// Attach it as an RX interrupt only.
	
	// Tell the imu to start sending in case it isn't doing that already. (The
	// IMU sends data upon receiving the character '4'. Note that this is not
	// the numerical value 4.
	p_device->putc('4');
}

IMU::~IMU()
{
	if (p_device != NULL)
	{
		delete p_device; // prevents memory leaks
	}
	delete rx_buffer;
}

//Wrapper function to write a character to the IMU
void IMU::putc(char c)
{
	p_device->putc(c);
}

// Checks data integrity of buffer, then stores the IMU values into variables.
void IMU::parse()
{
	// Null terminate the buffer just in case (for string safety)
	linebuf[i_linebuf] = '\0';
	
	short temp[9];
	// Check data integrity of sscanf()'s results. sscanf() returns the number
	// of values that it was able to extract. In this case, there are 9 values
	// we are reading.
	// we <3 pointer arithmetic
	if (sscanf(linebuf, "\n\r$%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd#", 
		temp, temp+1, temp+2, temp+3, temp+4, temp+5, temp+6, temp+7, temp+8) == 9)
	{
		accX = temp[0];
		accY = temp[1];
		accZ = temp[2];
		gyrX = temp[3];
		gyrY = temp[4];
		gyrZ = temp[5];
		magX = temp[6];
		magY = temp[7];
		magZ = temp[8];
		
		// Indicate that we read data successfully.
		led3 = !led3;
	}
}


bool IMU::readable()
{
	return p_device->readable();
}

void IMU::getData()
{
	// While there is data to be read, or the buffer has overflowed.
	while (!rx_buffer->empty || rx_buffer->overflow)
	{
		// Throw a char into the buffer
		linebuf[i_linebuf++] = rx_buffer->readByte();
		led4 = !led4;
		// If we reached the end of a set of IMU data (by receiving '#')
		if (linebuf[i_linebuf - 1] == '#')
		{
			//terminate the string
			linebuf[i_linebuf] = '\0';
			parseNow = true;
			break;
		}
	}
	
	NVIC_EnableIRQ(UART3_IRQn);
	
	if (parseNow)
	{
		parse();

		// The IMU should be flat when you run this calibration.
		// Calculates the average values of all sensors in order to set biases.
		if (calibrationEnabled)
		{
			sumAccX += accX;
			sumAccY += accY;
			sumAccZ += accZ;

			sumGyrX += gyrX;
			sumGyrY += gyrY;
			sumGyrZ += gyrZ;

			sumMagX += magX;
			sumMagY += magY;
			sumMagZ += magZ;

			num++;
		}
		
		// Clear the line buffer
		i_linebuf = 0;
		
		parseNow = false;
	}
}

// Call to get direct (raw) access to the IMU when it screws itself up.
void IMU::directAccess()
{
	// Disable the interrupt for reading from the IMU
	NVIC_DisableIRQ(UART3_IRQn);
	
	p_pc->printf("Entering IMU direct access mode. To exit, you need to reset the mbed.\n\r");
	
	// Act as a passthrough between the IMU and the PC (BeagleBoard).
	while (true)
	{
		// Send a character from PC to IMU (if possible).
		if (p_pc->readable())
		{
			char in = p_pc->getc();
			if (in == 'q')
			{
				NVIC_EnableIRQ(UART3_IRQn);
				return;
			}
			
			p_device->putc(in);
		}
		// Send a character from IMU to PC (if possible).
		if (p_device->readable())
		{
			p_pc->putc(p_device->getc());
		}
	}
}

void IMU::setCalibrationEnabled(bool isEnabled)
{
	calibrationEnabled = isEnabled;
	// If we're turning calibration on, clear the variables that stored the old
	// calibration data.
	if (isEnabled)
	{
		sumAccX = sumAccY = sumAccZ = sumGyrX = sumGyrY = sumGyrZ = sumMagX = sumMagY = sumMagZ = num = 0;
	}
}

void rx_interrupt_imu()
{
	// Indicate that the interrupt was called.
	led2 = !led2;
	
	// While there are characters to be read from the IMU
	while (imu.p_device->readable())
	{
		//pc.putc(imu.buffer[imu.i_buffer_write]);
		NVIC_DisableIRQ(UART3_IRQn);
		imu.rx_buffer->writeByte(imu.p_device->getc());
		NVIC_EnableIRQ(UART3_IRQn);
		if (imu.rx_buffer->overflow)
		{
			NVIC_DisableIRQ(UART3_IRQn);
			break;
		}
	}
}
