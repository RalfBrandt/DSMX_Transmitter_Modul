//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

//98000BFE295413FE26A81BFE0154 //DSMX

//900005FF14AA09FF13540DFF00AA   //DSM2

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "serial.h"
#include "timer.h"
#include "CYRF6936.h"
#include "DSM.h"
#include "config.h"

//global data
BOOL HighRes; //true if incomming data is 11Bit
BOOL New_Ser_Data; //true if a new serial package is recived
BOOL ADD_CH_Map;	//true if channel nomber in serial stream are missing
WORD Ch_Data[6];  //channel data is alwasy 11Bit resolution
BYTE ModellNr;	//modell number from serial stream


//get the biggest channel number in data package
BYTE GetMaxCh(BYTE *max_pos)
{
	BYTE n;
	BYTE chanal;
	BYTE max=0;
	for (n=0;n<6;n++)
	{
		chanal=(ser_databuf[n*2+2]>>(HighRes?3:2))&0xF;
		if(ser_databuf[n*2+2]!=0xFF  && chanal>max)
		{
			*max_pos=n;
			max=chanal;
		}
	}		
return max;
}

//called from  serial if a full package is recived
void Ser_cb(void)
{
	SER_RESET()
	//manual inline
	//SetOutData();
	//void SetOutData(void)
	{
		static BYTE n;
		//loop thru the channel data 
		for(n=0;n<6;n++)
		{
		//Ch_Data is alwasy 11Bit so if incoming data is 10Bit we need to shift up by one bit		
		if(HighRes)
			Ch_Data[n]=((ser_databuf[n*2+2]<<8)|ser_databuf[n*2+3]);
		else
			Ch_Data[n]=((ser_databuf[n*2+2]<<8)|ser_databuf[n*2+3])<<1;
		}
		if(ADD_CH_Map)
			Ch_Data[n]|=(n<<11);
	}
	New_Ser_Data=TRUE;
}

void main(void)
{
	BYTE config;				//configuration

	//initialize everiting
	M8C_EnableGInt;		 		// Enable Global Interrupts
#ifdef DEBUG
	TX8SW_1_Start();
#endif
	SerialRX_Init();
	Timer_init();
#ifdef DEBUG
	TX8SW_1_CPutString("Hello");
#endif
	LED_1_Start();
	DSM_Init();
#ifndef DEBUG
	E2PROM_1_Start();
#endif
	New_Ser_Data=FALSE;
	SerialSetCallback(Ser_cb);
#ifdef DEBUG
	//DIAG_OUT1_Start();
#endif

	//wait for serial data
	while(!New_Ser_Data);
	New_Ser_Data=FALSE;
	//check format of serial data, do we have 10bit or 11Bit data
	
	//manual inline
	//DeterminBitCount();
	//void DeterminBitCount(void)
	{
		BYTE chanal;
		BYTE n;
		BYTE min=0xff;
		BYTE max=0;

		//try 11 Bits
		for (n=0;n<6;n++)
		{
			chanal=(ser_databuf[n*2+2]>>3)&0xF;
			if (chanal&&chanal<min)
				min=chanal;
			if(ser_databuf[n*2+2]!=0xFF  && chanal>max)
				max=chanal;
		}		
		if ((max-min)>=4)
			HighRes=TRUE;
		else 
			HighRes=FALSE;
	}
	//get model number from serail stream
	//range is from 0 to 15
	ModellNr=ser_databuf[1];
	
	
	DSM_SetRangeCheck(ser_databuf[0]&SPTX_RANGE_CHECK);

	
	//manual inline
	//Determin_Mode();
	//void Determin_Mode(void)
	{
		BYTE max2;
		BYTE max_pos;
		BYTE max_pos2;

		//get highest channel found in data stream
		BYTE Max_Ch=GetMaxCh(&max_pos);
		//wait for next serial data
		while(!New_Ser_Data);
		//get highest channel found in data stream
		max2=GetMaxCh(&max_pos2);
		//if they are different we have more that 6 channel 
		if(max2!=Max_Ch && Max_Ch && max2)
			DSM_Is_11ms=TRUE;

		//update max	
		if(max2>Max_Ch)
			{
			Max_Ch=max2;
			max_pos=max_pos2;
			}
		//if channel numbers are missing in data stream we need to add them
		if(!Max_Ch)
			ADD_CH_Map=TRUE;

		//determin Mode
		DSM_Is_DSMX=((ser_databuf[0]&SPTX_DSMX_MODE)==SPTX_DSMX_MODE);
		DSM_Is_Forced=!((ser_databuf[0]&SPTX_NORM_MODE)==SPTX_NORM_MODE);
	
		//DSMX is always 11Bit
		if(DSM_Is_DSMX || HighRes)
			DSM_Is_11Bit=TRUE;

		//update channel number send out while bind 
		if(Max_Ch>5)
			num_channels=Max_Ch+1;
		dsm_Max_pos=max_pos;
			
#ifdef DEBUG
		TX8SW_1_CPutString(" C:");
		TX8SW_1_PutSHexByte(DSM_Is_DSMX);
		TX8SW_1_PutSHexByte(DSM_Is_11Bit);
		TX8SW_1_PutSHexByte(DSM_Is_11ms);
		TX8SW_1_PutSHexByte(HighRes);
		TX8SW_1_PutSHexByte(ADD_CH_Map);
		TX8SW_1_PutSHexByte(Max_Ch);
		TX8SW_1_PutSHexByte(dsm_Max_pos);
#endif
	}
	
	if(ser_databuf[0]&SPTX_BIND_MODE)
	{
#ifdef DEBUG
		TX8SW_1_CPutString("BIND ");
#endif
		DSM_initialize_bind();
		do
			{
			DSM_Cyclic();
			}
		//wait till binding is done
		while(!DSM_BIND_DONE());
		
		New_Ser_Data=FALSE;
		
		//save configuration to FLASH
		config=(DSM_Is_11Bit?OPTION_11BIT:0)|(DSM_Is_11ms?OPTION_11MS:0)|(DSM_Is_DSMX?OPTION_DSMX:0);
#ifdef DEBUG
		TX8SW_1_PutSHexByte(config);
		TX8SW_1_CPutString("SAVE ");
#endif
		SaveConfig(config,ModellNr);
	}
//read config from flash
#ifdef DEBUG
	config=(DSM_Is_11Bit?OPTION_11BIT:0)|(DSM_Is_11ms?OPTION_11MS:0)|(DSM_Is_DSMX?OPTION_DSMX:0);
#else
	config=ReadConfig(ModellNr);
#endif

#ifdef DEBUG
	TX8SW_1_CPutString("RUN ");
	TX8SW_1_PutSHexByte(config);
	//DIAG_OUT1_Off();
#endif
	//determin config based on value from flash
	DSM_Is_11ms=((config&OPTION_11MS)==OPTION_11MS);
	DSM_Is_11Bit=((config&OPTION_11BIT)==OPTION_11BIT);
	DSM_Is_DSMX=((config&OPTION_DSMX)==OPTION_DSMX);
	//prepare transmitting
	DSM_LoadTransmitMode();
	New_Ser_Data=FALSE;
	while (1)
	{
	DSM_Cyclic();

	if(New_Ser_Data)
		{
		New_Ser_Data=FALSE;
		
		DSM_NewSerialData();
	
		//update TX power
		DSM_SetRangeCheck(ser_databuf[0]&SPTX_RANGE_CHECK);

		LED_1_Invert(); 
		}
	}
}
