#include "config.h"

extern CHAR  E2PROM_1_bE2Write(WORD wAddr, BYTE * pbData, WORD wByteCount, CHAR cTemperature);
extern void  E2PROM_1_E2Read(WORD wAddr, BYTE * pbDataDest, WORD wByteCount);


RAM_D mem;
#define buf mem.buf

void SaveConfig(BYTE options,BYTE Nr)
{
Nr=Nr&0xF;
#ifndef DEBUG
E2PROM_1_E2Read(0,buf,32);
#endif
buf[Nr*2]=CONFIG_ID;
buf[Nr*2+1]=options;
#ifndef DEBUG
E2PROM_1_bE2Write(0,buf,64,25);
#endif
}

BYTE ReadConfig(BYTE Nr)
{
Nr=Nr&0xF;
#ifndef DEBUG
E2PROM_1_E2Read(0,buf,32);
#endif
if(buf[Nr*2]!=CONFIG_ID)
	{
		return 0;
	}
else
	{
		return buf[Nr*2+1];
	}
}



