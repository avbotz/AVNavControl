#ifndef BUFFER_H_
#define BUFFER_H_

struct CircularBuffer {
public:
	CircularBuffer(int size);
	~CircularBuffer(int size);
	
	bool empty;
	bool overflow;
	char readNextByte();	//read the next byte from the buffer and increment the read counter
	void writeNextByte(char c);	//write the next byte to the buffer and increment the write counter
	bool hasData(int n);	//returns whether the buffer has n unread bytes
	
private:
	char* buffer;
	int i_read, i_write, size;
};

#endif