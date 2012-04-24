/* The tcp client program. Performs all relaying operations, such as forwarding messages */

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
#include <tcp_client.h>

int sock, bytes;
fd_set in_set;
fd_set out_set;
int zig_fd;
unsigned int command;

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
void init_selector()
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
		perror("FD_SET(sock)");
    /** The relay-robot socket */
	if(zig_fd != -1)
	{
		FD_SET(zig_fd , &in_set);
		FD_SET(zig_fd , &out_set);
	}

	else
		perror("FD_SET(zig_sock)");
}

/** The selector needs to be initialized every time the two communication sockets are polled */
int chk_select()
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
			return 0;
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
				zgb_send(command);
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

int send_to_server(int* _send)
{
	int response = write(sock, _send, sizeof(_send));
	return response;
}


int receive_from_server(unsigned int* _recv)
{
	int res = read(sock, _recv, sizeof(int));
	return res;
}

int close_server_connection()
{
	return close(sock);
}

int zig_init()
{
	printf("Connecting to robot...");
	printf("DEBUG flag is %d\r\n", DEBUG);
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
        return 1;
    }
}

void zgb_send(int output)
{
	if(zgb_tx_data(output) == 0) 
	{
		printf( "\r\nFailed to transmit\r\n" );
	}
}

unsigned int cmd_assemble(unsigned int time, unsigned int speed, unsigned int cmd)
{
	unsigned int command = time + (speed << 4) + (cmd << 16);
	return command;

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
