/**
 * All relaying stuff is done in the tcp_client. This file is used to start up the relay and close it */

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <tcp_client.h>
#include </usr/include/ncurses.h>
#include <stdlib.h>

int main()
{
	if(zig_init() && connect_to_server())
	{
        printf("Connected.\n");
        while(1)
        {

        	int stat = chk_select();
        	if(stat <= 0) // Received TERM signal, shutdown 
        	{
        		break;
        	}

        }
        	printf("Received TERM signal. Closing... \n");
        	close_server_connection();
        	if(!DEBUG)
        		close_zig_conn();

    }
	return 0;

}
