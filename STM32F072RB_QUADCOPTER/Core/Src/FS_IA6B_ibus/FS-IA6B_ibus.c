/*
 * FS-IA6B_ibus.c
 *
 *  Created on: Dec 19, 2023
 *      Author: uzeyir
 */


/*
 * FS-iA6B.c
 *
 *  Created on: 2019. 9. 15.
 *      Author: Administrator
 */

#include "FS_IA6B_ibus/FS-IA6B_ibus.h"
#include "main.h"
FSiA6B_iBus iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len)
{
	unsigned short chksum = 0xffff;

	for(int i=0;i<len-2;i++)
	{
		chksum = chksum - data[i];
	}

	return ((chksum&0x00ff)==data[30]) && ((chksum>>8)==data[31]);
}

void iBus_Parsing(unsigned char* data, FSiA6B_iBus* iBus)
{
	iBus->RH = (data[2] | data[3]<<8) & 0x0fff;
	iBus->RV = (data[4] | data[5]<<8) & 0x0fff;
	iBus->LV = (data[6] | data[7]<<8) & 0x0fff;
	iBus->LH = (data[8] | data[9]<<8) & 0x0fff;
	iBus->SwA = (data[10] | data[11]<<8) & 0x0fff;
	iBus->SwC = (data[12] | data[13]<<8) & 0x0fff;

	iBus->FailSafe = (data[13] >> 4);
}

unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus)
{
	return iBus->FailSafe != 0;
}

void IbusData(uint16_t* ibus_data, int IBUS_USER_CHANNELS, uint8_t* rxBuf)
{
	int ch_index,bf_index;
	for(ch_index = 0, bf_index = 2; ch_index < IBUS_USER_CHANNELS; ch_index++, bf_index += 2)
		ibus_data[ch_index] = rxBuf[bf_index + 1] << 8 | rxBuf[bf_index];
}
