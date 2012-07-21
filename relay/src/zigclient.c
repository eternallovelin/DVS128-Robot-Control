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

/* The zigbee client program. Performs all relaying operations, such as connecting to the robot and forwarding 
messages between the robot and the image processor. Two types of initialization are available; with and without 
keyboard. The first is used for the main level of operation, while the keyboard mode is used when running keyboard 
level. A socket select struct is used to access the two established sockets and determine if any messages are pending. */

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <zigbee.h>
#include <zgb_hal.h>
#include <zigclient.h>
#include </usr/include/ncurses.h>

int sock, bytes;
fd_set in_set;
fd_set out_set;
int zig_fd;
unsigned int command;

/** Initialize in main mode */
int initialize()
{
	printf("Connecting to robot...");
	// Open device
    if(!NO_ZIGBEE) /** NO_ZIGBEE is used for debugging the pc-relay communication */
    {
        if( zgb_initialize(DEFAULT_DEVICEINDEX, &zig_fd) == 0 )
        {
          printf( "Failed to open Zig2Serial!\n" );
          return 0;
        }
        else
        {
          printf( "Succeed to open Zig2Serial!\r\n" );
          return 1;
        }
    }
    else
    {
        printf("DEBUG flag is %d\r\n", DEBUG);
        return 1;
    }
}

/* Initialize in Keyboard mode */
int initialize_with_keyboard()
{
	keyboard();
	int success = initialize();
	refresh();
	return success;
}

/* Connect to the image receiver and controller components */
int connect_to_server()
{
	printf("Connecting to server..");
    struct hostent *host;
    struct sockaddr_in server_addr;  
    host = gethostbyname(HOSTNAME);
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Socket");
        return 0;
    }
    server_addr.sin_family = AF_INET;     
    server_addr.sin_port = htons(PORT);   
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    bzero(&(server_addr.sin_zero),8); 
    if (connect(sock, (struct sockaddr *)&server_addr,
                sizeof(struct sockaddr)) == -1) 
    {
        perror("Connect");
        return 0;
    }
    else
    {
        return 1;
    }
}

/** see http://www.gnu.org/software/libc/manual/html_node/Waiting-for-I_002fO.html */
int init_selector()
{
	FD_ZERO (&in_set);
	FD_ZERO (&out_set);
    /** The pc-relay socket */
	if( sock != -1)
	{
		FD_SET(sock, &in_set);
		FD_SET(sock, &out_set);
	}
	else
	{
		return 0;
	}
    /** The relay-robot socket */
	if(zig_fd != -1)
	{
		FD_SET(zig_fd , &in_set);
		FD_SET(zig_fd , &out_set);
	}
	else
	{
		return 0;
	}
	return 1;
}

/** The selector needs to be initialized every time the two communication sockets are polled */
int check_select()
{
	init_selector();
	int stat = select(FD_SETSIZE, &in_set,&out_set, NULL, NULL);
    /** fail */
	if(stat < 0)
	{
		perror("select");
		exit (EXIT_FAILURE);
	}
	else if(stat == 0)
	{
		return 0;
	}
	if(FD_ISSET(sock,&in_set))
	{
		FD_CLR(sock, &in_set);
		receive_from_server(&command);
		if(command == 0xFFFFFFFF) /** This is the TERM command: shutdown */
		{
			return 0;		
		}
		printf("Received from server the following command: %d\n", command);
		if(FD_ISSET(zig_fd, &in_set))
		{
			FD_CLR(zig_fd, &in_set);
			printf("Also receiving from robot. Ignoring server command...\n");
			int rbt_recv = zgb_recv();
			if(FD_ISSET(sock, &out_set))
			{
				FD_CLR(sock, &out_set);
				printf("Sending %d to server\n",rbt_recv);
				send_to_server(&rbt_recv);
			}
			else
			{
				printf("Server socket not ready to write!\n");
			}
		}
		else
		{
			if(FD_ISSET(zig_fd,&out_set))
			{
				FD_CLR(zig_fd, &out_set);
				printf("Sending %d to robot..\n",command);
				zgb_send16(command);
			}
			else
			{
				printf("Socket not ready to write to robot!\n");
			}
		}
	}
	if(FD_ISSET(zig_fd, &in_set))
	{
		FD_CLR(zig_fd, &in_set);
		int rbt_recv = zgb_recv();
		if(FD_ISSET(sock, &out_set))
		{
			FD_CLR(sock, &out_set);
			printf("Sending %d to server\n",rbt_recv);
			send_to_server(&rbt_recv);
		}
		else
		{
			printf("Server socket not ready to write!\n");
		}
	}
	return 1;
}

/** Simple write-to-socket operation, sock is the established socket between the relay and the Java controller */
int send_to_server(int* _send)
{
	int response = write(sock, _send, sizeof(_send));
	return response;
}

/* Read-from-socket operation, sock is the established socket between the relay and the Java controller */
int receive_from_server(unsigned int* _recv)
{
	int res = read(sock, _recv, sizeof(int));
	return res;
}

int close_server_connection()
{
	return close(sock);
}

/** Uses 16-bit unsigned integer, uint16. Must be careful with integer formats, as I'm dealing with an 8-bit microcontroller 
(the cm510), the Java 16-bit integer type and the C 16-bit unsigned integer type  The cm510 controller assembles the 16-bit integer 
through a loop, as the ingteger is split into 3 bytes */
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

/** Assembles a command to be executed by the robot */
uint16_t assemble16(uint16_t time16, uint16_t speed16, uint16_t cmd16)
{
	uint16_t command16 = speed16 + (time16 << 13) + (cmd16 << 10);
	return command16;
}

int zgb_recv()
{
	while(zgb_rx_check() != 1);
	{
		int response = 0;
		response = zgb_rx_data();
		return response;
	}
}

void close_zig_conn()
{
	zgb_terminate();
}

void keyboard()
{
	initscr();		// Start ncurses mode
	raw();			// Line buffering disabled
	keypad(stdscr, TRUE);	// get F1, F2,etc.
	cbreak();
	nonl();
	noecho();
	refresh();
}
