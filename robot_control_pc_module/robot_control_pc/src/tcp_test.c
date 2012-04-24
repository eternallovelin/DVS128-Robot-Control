#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <tcp_client.h>
#include </usr/include/ncurses.h>
#include <stdlib.h>

unsigned int command;

int main()
{
	if(zig_init() && connect_to_server())
	{
        printf("Connected.\n");
        while(1)
        {

        	int stat = chk_select();
        	if(stat <= 0)
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
