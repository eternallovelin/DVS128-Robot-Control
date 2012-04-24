#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/sleep.h>
#include "zigbee.h"
#include "dynamixel.h"
#include "dynamixel_control.h"

volatile unsigned int msg;
volatile unsigned int count,header_received; // Don't really need the header_Received variable anymore
volatile unsigned int cmd, speed, time;
volatile int flag;

int main(void)
{
	msg = count = flag = header_received = 0;
	
	DDRC  = 0x7F;

	PORTD &= ~0x80;	//PORT_LINK_PLUGIN = 0;   // no pull up
	PORTD &= ~0x20;	//PORT_ENABLE_RXD_LINK_PC = 0;
	PORTD |= 0x40;	//PORT_ENABLE_RXD_LINK_ZIGBEE = 1;

	zgb_initialize( 0 ); // Not using device index
	dyn_init();



	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();	// Interrupt Enable	
	
	while(1)
	{
		sleep_mode();
		if(flag)
		{
			dyn_exec(cmd,speed,time);
			flag = 0;
		}
	}

	return 0;
}

/* Interrupt handler for incoming commands */
ISR(USART1_RX_vect)
{	
	unsigned int recv = 0;
	recv = UDR1;
	PORTC = ~(1<<6);
	switch(count)
	{
		case 0: // It's the header, check if so
			if(recv == 0xff)
			{
					header_received = 1;
					count++;
			}			
			recv = 0;
			break;
		case 1:
			if(header_received == 1)
			{
				msg += recv;
			}
			count++;
			recv = 0;
			break;
		case 2:
			if(header_received == 1)
			{
				msg += (recv << 8);
				speed = msg & 0x3FF;
				cmd = (msg >> 10) & 0x7;
				time = (msg >> 13) & 0x7;
				if(speed > 1023)
					speed = 1023;
				if(time > 4)
					time = 4;
				if(cmd == 0)
				{
					cancel_command();
				}
				else
				{
					flag = 1;
				}
			}
			msg = 0;
			count = 0;
			recv = 0;
			break;
		
	}
	PORTC = (1<<0);
	
}
