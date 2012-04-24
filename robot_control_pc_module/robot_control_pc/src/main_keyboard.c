#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include </usr/include/ncurses.h>
#include <stdlib.h>
#include <zig_control.h>
#include <zigbee.h>

int main()
{
	char input;
    unsigned int response = 0;
    uint16_t command;
	if(init())
	{
		printf("Starting up server.");
		refresh();
		while(1)
		{
			printf("Send a command: w(Move Forward), s(Move Backward), "
					"a(Turn Forward Left), d(Turn Forward Right),"
					"z(Turn Backward Left), x(Turn Backward Right), "
					"q(Quit), c(Cancel last command), "
					"y(Check Robot Status)\r\n");
			refresh();

			input = wgetch(stdscr);

			if(input == 'q')
			{
				printf("Quitting...\n");
				refresh();
				break;
			}
			else if(input == 'c')
			{
				printf("Cancelling last command...\r\n");
				refresh();
				command = assemble16(0,0,0);
				zgb_send16(command);
			}
			else if(input == 'y')
			{
				printf("Checking robot status... \r\n");
				refresh();
				command = assemble16(0,0,0);
				zgb_send16(command);
			}
			else
			{
				printf("Command: %c\r\n",input);
				int cmd =input;
				switch(cmd)
				{
				  case 'w':
				    cmd = 1;
				    break;
				  case 's':
				    cmd = 2;
				    break;
				  case 'a':
				    cmd = 3;
				    break;
				  case 'd':
				    cmd = 4;
				    break;
				  case 'z':
					cmd = 5;
					break;
				  case 'x':
					cmd = 6;
					break;
				 }
				int speed = 0;
				int _time = -1;
				while(speed == 0)
				{
					printf("Now, enter movement speed value:\r\n1(40),\r\n2(80),"
							"\r\n3(120),\r\n4(200),\r\n5(450),\r\n6(600),\r\n7(760),\r\n8(900),\r\n9(1023)\r\n");
					refresh();
					input = wgetch(stdscr);
					int temp = atoi(&input);
					switch(temp)
					{
						case 1:
							speed=40;
							break;
						case 2:
							speed=80;
							break;
						case 3:
							speed=120;
							break;
						case 4:
							speed=200;
							break;
						case 5:
							speed=450;
							break;
						case 6:
							speed=600;
							break;
						case 7:
							speed=760;
							break;
					        case 8:
							speed=900;
							break;
						case 9:
							speed=1023;
							break;
						default:
							speed=0;
							break;
					}
				}
				while(_time == -1)
				{
					printf("Now, enter time for execution of command:\r\n1(1),\r\n2(2),"
												"\r\n3(3),\r\n4(4),\r\n5(5),\r\n6(6),\r\n7(7),\r\n8(8),\r\n9(9)\r\n");
					refresh();
					input = wgetch(stdscr);
					int temp = atoi(&input);
					if(0 <= temp && temp <=9)
						_time = temp;
					else
						_time = -1;
				}
				printf("Command is: %d; speed is: %d; time is: %d\r\n", cmd, speed, _time);
				printf("Sending...\r\n");
				refresh();
				command = assemble16(_time, speed, cmd);
				printf("Output is: %d\r\nSize: %d\r\n", command,sizeof(command));
				refresh();
				zgb_send16(command);

			}
			
            /*while(1)
            {
                if(zgb_rx_check() == 1)
                {
                    response = zgb_rx_data();
                    printf("Received: %d\r\n",response);
                    break;
                }
                else    
                {
                    printf("NO PACKET RECEIVED...\r\n");
                    refresh();
                }
                sleep(1);
            }*/
		
		}
		
	}
		

	endwin();
	
	// Close device
	zig_close();
	return 0;
}
