//##########################################################
//##                      IVAN KONSTANTINOV               ##
//##        MAIN CONTROLLER FOR CM510					  ##
//##                                           20.06.2011 ##
//##########################################################

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "zigbee.h"

#include <string.h>

#include "dynamixel_control.h"
#include "dynamixel.h"

volatile int stop_delay=0;
volatile int is_executing_cmd = 0;

int ids[4] = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT}; 
	
int dyn_init()
{
	int dxl_res = dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
	if(dxl_res == 0) // Failed to init dxl 
		return DXL_FAIL;
	set_wheel_mode(FRONT_LEFT);
	set_wheel_mode(FRONT_RIGHT);
	set_wheel_mode(BACK_LEFT);
	set_wheel_mode(BACK_RIGHT);
	return INIT_SUCCESS;

}

int set_wheel_mode(int id)
{
	int comm_status;		
	dxl_write_word(id, CW_ANGLE_LIMIT_L, 0);
	comm_status = dxl_get_result();
	
	dxl_write_word(id, CCW_ANGLE_LIMIT_L, 0);
	comm_status = dxl_get_result();

	return CW_CCW_ANGLE_SUCCESS;

}

/** Execute the passed command */
int dyn_exec(unsigned int cmd,unsigned int speed,unsigned int time)
{
	stop_delay = 0;
	switch(cmd)
	{
		case 1: // Forward
			is_executing_cmd = 1;
			dyn_forward(speed, time);
			is_executing_cmd = 0;
			break;
		case 2: // Backward
			is_executing_cmd = 1;
			dyn_backward(speed, time);
			is_executing_cmd = 0;
			break;
		case 3: // Turn forward Left 
			is_executing_cmd = 1;
			dyn_turn_left(1,speed, time);
			is_executing_cmd = 0;
			break;
		case 4: // Turn forward right 
			is_executing_cmd = 1;
			dyn_turn_right(1,speed,time);
			is_executing_cmd = 0;
			break;
		case 5: // Turn backward left
			is_executing_cmd = 1;
			dyn_turn_left(0,speed,time);
			is_executing_cmd = 0;
			break;
		case 6: // Turn backward right
			is_executing_cmd = 1;
			dyn_turn_right(0, speed, time);
			is_executing_cmd = 0;
			break;
		default:
			break;
			
	}
	return 0;
}

/**
  * Sends a broadcast message to the four servo motors. The values represent the movement speed, 
  * ranging 0-1023 and comprising two bytes
  */
void send_dxl_broadcast(int front_left, int front_right, int back_left, int back_right)
{
	int index;
	int values[4] = {front_left, front_right, back_left, back_right};
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, MOVING_SPEED_L);
	dxl_set_txpacket_parameter(1, 2);
	for (index=0;index<4;index++) // Fill with parameters for each servo moror 	
	{
		dxl_set_txpacket_parameter(2+3*index, ids[index]);
		dxl_set_txpacket_parameter(2+3*index+1, dxl_get_lowbyte(values[index]));
		dxl_set_txpacket_parameter(2+3*index+2, dxl_get_highbyte(values[index]));
	}
	dxl_set_txpacket_length((2+1)*4+4);
	dxl_txrx_packet();
}

void dyn_stop()
{
	dxl_write_word(BROADCAST_ID, MOVING_SPEED_L, 0);
}

void cancel_command()
{
	PORTC = ~(1<<1);
	//int status = check_if_moving();
	PORTC = ~(1<<2);
	if(is_executing_cmd == 1)
	{
		PORTC = ~(1<<3);
		dyn_stop();
		PORTC = ~(1<<4);
		stop_delay = 1;
	}
	else
	{
		PORTC = ~(1<<5);
		confirm();
	}
	PORTC = ~(1<<6);
}

void dyn_forward(unsigned int speed,unsigned int time)
{
	
	send_dxl_broadcast(CCW_SPEED_L+speed, CW_SPEED_L+speed, CCW_SPEED_L+speed, CW_SPEED_L+speed);
	delay(time);
	if(time != 0)
	{
		dyn_stop();
		confirm();
	}
	
}

void dyn_backward(unsigned int speed,unsigned int time)
{
	send_dxl_broadcast(CW_SPEED_L + speed,CCW_SPEED_L + speed,CW_SPEED_L + speed,CCW_SPEED_L + speed );
	delay(time);

	if(time != 0)
	{
		dyn_stop();
		confirm();
	}
}

void dyn_turn_left(int direction, unsigned int speed,unsigned int time)
{
	switch(direction)
	{
		case 1:
			send_dxl_broadcast(CCW_SPEED_L + (speed)*TURN_WEIGHT, CW_SPEED_L + speed, CCW_SPEED_L + (speed)*TURN_WEIGHT,CW_SPEED_L + speed);
			break;
		case 0:
			send_dxl_broadcast(CW_SPEED_L + (speed)*TURN_WEIGHT, CCW_SPEED_L + speed, CW_SPEED_L + (speed)*TURN_WEIGHT, CCW_SPEED_L + speed);
			break;
	}
	delay(time);
	if(time != 0)
	{
		dyn_stop();
		confirm();
	}
}

void dyn_turn_right(int direction, unsigned int speed,unsigned int time)
{
	switch(direction)
	{
		case 0:
			send_dxl_broadcast(CW_SPEED_L + speed,CCW_SPEED_L + (speed)*TURN_WEIGHT,CW_SPEED_L + speed,CCW_SPEED_L + (speed)*TURN_WEIGHT);
			break;
		case 1:
			send_dxl_broadcast(CCW_SPEED_L + speed,CW_SPEED_L + (speed)*TURN_WEIGHT,CCW_SPEED_L + speed,CW_SPEED_L + (speed)*TURN_WEIGHT);
			break;
	}
				
	delay(time);
	
	if(time != 0)
	{
		dyn_stop();
		confirm();
	}
}

int check_if_moving()
{
	int front_left = dxl_read_word (FRONT_LEFT, MOVING );
	int front_right = dxl_read_word( FRONT_RIGHT, MOVING );
	int back_left = dxl_read_word( BACK_LEFT, MOVING );
	int back_right = dxl_read_word( BACK_LEFT, MOVING );
	int is_moving = (front_left != 0) | (front_right != 0) | (back_left != 0) | (back_right != 0);
	
	return is_moving;
}

/* Must check what time value has been passed, since the max delay time is 4192 milliseconds (4.192 seconds)
If the time value is larger, several delay_ms commands will be passed, according to the value */
void check_delay(int* delays, unsigned int time)
{
	// First value is the number of delay_ms commands to be issued,
	//  the second value is the delay amount for the last delay_ms
	delays[1] = time % 4; // Get the remainder, that is the time value for the last delay_ms to be called
	delays[0] = ((time - delays[1])/4); // Get the number of full delay_ms commands
	
}

void delay(unsigned int time)
{

	if(time == 0)
	{
		while(stop_delay == 0)
		{
			_delay_ms(1000);
		}
		stop_delay = 0;
	}
	else
	{
		if(time > 9)
		time = 3; // wrap around. 
		unsigned int num_times;
		for(num_times=0;num_times<time;num_times++)
		{
			if(stop_delay == 1)
			{
				stop_delay = 0;
				break;
			}
			else
			{
				_delay_ms(1000);
			}
		}
	}
	
}

void set_stop_delay(int flag)
{
	stop_delay = flag;
}

void confirm()
{
	zgb_tx_data(1);
}
