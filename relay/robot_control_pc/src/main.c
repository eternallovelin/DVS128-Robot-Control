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
