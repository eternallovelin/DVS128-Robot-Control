#include "zgb_hal.h"
#include "zigbee.h"
#include <avr/io.h>

unsigned char gbRcvPacket[DATA_SIZE];
unsigned char gbRcvPacketNum;
unsigned long gwRcvData;
unsigned char gbRcvFlag;

int zgb_initialize( int devIndex )
{
	if( zgb_hal_open( devIndex, 57600 ) == 0) // Always fixed baudrate
		return 0;

	gbRcvFlag = 0;
	gwRcvData = 0;
	gbRcvPacketNum = 0;
	return 1;
}

void zgb_terminate(void)
{
	zgb_hal_close();
}

int zgb_tx_data(unsigned long data)
{
	unsigned char packet[DATA_SIZE];
	unsigned char mask = 0xff;
	// First byte is header; 0xff
	packet[0] = 0xff;
	// Next 4 bytes are data
	packet[1] = data & mask;
	packet[2] = (data >> 8) & mask;
	packet[3] = (data >> 16) & mask;
	packet[4] = (data >> 24) & mask;
	
	if( zgb_hal_tx( packet, DATA_SIZE) != DATA_SIZE )
		return 0;

	return 1;
}

int zgb_rx_check(void)
{
	int RcvNum;
	int i, j;

	if(gbRcvFlag == 1)
		return 1;
	// Fill packet buffer
	if(gbRcvPacketNum < DATA_SIZE)
	{
		RcvNum = zgb_hal_rx( &gbRcvPacket[gbRcvPacketNum], (DATA_SIZE - gbRcvPacketNum) );
		if( RcvNum != -1 )
			gbRcvPacketNum += RcvNum;
	}
	// Find header
	
	if(gbRcvPacketNum >= 1)
	{
		for( i=0; i<gbRcvPacketNum; i++ )
		{
			if(gbRcvPacket[i] == 0xff)
			{
				break;
			}
		}
		if(i > 0)
		{
			if(i == gbRcvPacketNum)
			{
				// Can not find header
				if(gbRcvPacket[i - 1] == 0xff)
					i--;
			}

			// Remove data before header
			for( j=i; j<gbRcvPacketNum; j++)
			{
				gbRcvPacket[j - i] = gbRcvPacket[j];
			}
			gbRcvPacketNum -= i;
		}
	}
	// Verify packet
	if(gbRcvPacketNum == DATA_SIZE)
	{
		if(gbRcvPacket[0] == 0xff)
		{
			gwRcvData = gbRcvPacket[1] + ((unsigned long)gbRcvPacket[2]<<8) + ((unsigned long)gbRcvPacket[3] << 16) + ((unsigned long)gbRcvPacket[4] << 24);
			gbRcvFlag = 1;
		}

		gbRcvPacket[0] = 0x00;
		gbRcvPacketNum = 0;
	}
	return gbRcvFlag;
}

long zgb_rx_data(void)
{
	gbRcvFlag = 0;
	return gwRcvData;
}
