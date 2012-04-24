#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "serial.h"
#include "serial_control.h"

void init()
{
	serial_initialize(57600);	

	printf( "\n\nSerial Comm. example for CM-510\n\n" );

}	
