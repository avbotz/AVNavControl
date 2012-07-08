#ifndef PC_H
#define PC_H

#include <mbed.h>

#include "analog.h"
#include "pid.h"

#define PC_BUFFER_SIZE 1024

struct avnav {
	char byte1;
	char byte2;
};

void rx_interrupt_pc();
void tx_interrupt_pc();
void send_status_pc();

class PC {
public:
	PC(PinName tx, PinName rx, int baud);

	avnav encode_avnav(int data);
	int decode_avnav(avnav data);
	void send_message(const char* message);

	void putc(char c); //add c to the write FIFO

	char readPC();

	int desired_heading, desired_depth, desired_power;

	Serial* p_device;
	Ticker* pc_ticker;

	bool tx_empty;
	bool rx_overflow;

private:
	friend void rx_interrupt_pc();
	friend void tx_interrupt_pc();
	friend void send_status_pc();
	
	char* mes;
	int message_length;
	char tx_buffer[PC_BUFFER_SIZE], rx_buffer[PC_BUFFER_SIZE];
	int i_tx_read, i_tx_write;
	int i_rx_read, i_rx_write;

};

extern bool debug;
extern PC pc;

extern bool isAlive;

#endif
