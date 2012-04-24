/**	This file is part of DVS128-Robot-Control.

    DVS128-Robot-Control is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DVS128-Robot-Control is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * robot_control.c
 *
 *  Created on: Sep 22, 2011
 *      Author: vanxa
 */
#include <zig_control.h>
#include </usr/include/ncurses.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <zigbee.h>
#include <zgb_hal.h>

fd_set set;
int zig_fd;

int init()
{
	init_keyboard();
	if( zgb_initialize(DEFAULT_DEVICEINDEX, &zig_fd) == 0 )
	{
	  printf( "Failed to open Zig2Serial!\n" );
	  refresh();
	  return 0;
	}
	else
	{
	  printf( "Succeed to open Zig2Serial!\r\n" );
	  refresh();
          return 1;
	}
}
void init_keyboard()
{
	initscr();		// Start ncurses mode
	raw();			// Line buffering disabled
	keypad(stdscr, TRUE);	// get F1, F2,etc.
	cbreak();
	nonl();
	noecho();
	refresh();
}

void zgb_send(int output)
{
	if(zgb_tx_data(output) == 0)
	{
		printf( "\r\nFailed to transmit\r\n" );
		refresh();
	}
}

uint16_t assemble16(uint16_t time16, uint16_t speed16, uint16_t cmd16)
{
	uint16_t command16 = speed16 + (time16 << 13) + (cmd16 << 10);
    printf("Command is %d\r\n",command16);
    refresh();
	return command16;

}

void receive(unsigned int* response)
{
	int res = chk_select(10);
	if (res == 0)
	{
		printf("Timeout\r\n");
	}
	else if(res == 2)
	{
		while(zgb_rx_check() != 1);
        {
            *response = zgb_rx_data();
            printf("Response from robot: %d\r\n", *response);
            refresh();
        }
	}
	else if(res == 1)
	{
		printf("hmmmm\r\n");
		refresh();
	}

}

void init_selector()
{
	FD_ZERO (&set);
    if(zig_fd != -1)
    	FD_SET(zig_fd , &set);
    else
    {
        printf("FD_SET(zig_sock) error\n");
        refresh();
    }
            
}

int chk_select(unsigned int timeout_seconds)
{
	init_selector();
	struct timeval timeout;
	timeout.tv_sec = timeout_seconds;
	timeout.tv_usec = 0;

	int i;
		int res = 0;
		if(select(4, &set,NULL, NULL, &timeout) < 0)
		{
			perror("select");
			exit (EXIT_FAILURE);
		}
		for(i = 0; i < 4; ++i )
		{
			if (FD_ISSET (i, &set))
			{
				if(i == zig_fd)
				{
					res = 2;
				}
			}
		}
		return res;

}

void zig_close()
{
	zgb_terminate();
}

int zgb_send16(uint16_t data)
{
    unsigned char packet[3];
    unsigned char mask = 0xff;
    packet[0] = mask;
    packet[1] = data & mask;
    packet[2] = (data >> 8) & mask;

    if(zgb_hal_tx(packet, 3) != 3)
        return 0;
    return 1;
}
