#include "buffer.h"

CircularBuffer::CircularBuffer(int size)
{
	buffer = NULL;
	buffer = new char[size];
	if (buffer == NULL) {
		//TODO: ERROR OUT OF MEMORY
	}
	this.size = size;
	i_read = i_write = 0;
	empty = true;
	overflow = false;
}

CircularBuffer::~CircularBuffer()
{
	delete buffer;
}

char CircularBuffer::readNextByte()
{
	char c = buffer[i_read];
	i_read = (i_read + 1) % size;
	if (i_write == i_read) {
		empty = true;
	}
	//in reality not really, but lets pretend it does
	//becase if you're a whole buffer behind you're FUBAR already
	overflow = false;
}

void CircularBuffer::writeNextByte(char c)
{
	buffer[i_write] = c;
	i_write = (i_write + 1) % size;
	if (i_write == i_read) {
		overflow = true;
	}
	//after we write we can never be empty
	empty = false;
}

bool CircularBuffer::hasData(int n)
{
	//if reading n bytes does not cause us to loop around to the front of the buffer
	//just add n to figure out the last byte that will need to be read
	//otherwise, we have to loop around to the front of the buffer to check the last
	//byte that will be read
	return ((i_read + n) < size)?((i_read + n) <= i_write):((i_read + n) % size <= i_write);
}
