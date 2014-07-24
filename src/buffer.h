#ifndef BUFFER_H_
#define BUFFER_H_

#include "mbed.h"

struct CircularBuffer
{
public:
	CircularBuffer(int size);
	~CircularBuffer();
	
	bool empty;
	bool overflow;
	char readByte();	//read the next byte from the buffer and increment the read counter
	void writeByte(char c);	//write the next byte to the buffer and increment the write counter
	bool hasData(int n) const;	//returns whether the buffer has n unread bytes
	char peek(int n) const;		//returns what is n bytes ahead of the current read index
	
	int i_read, i_write, size;
private:
	char* buffer;
	
};

#endif
