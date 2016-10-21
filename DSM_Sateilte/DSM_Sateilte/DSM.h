#ifndef DSM_H
#define DSM_H
#include "cyrf6936.h"
#include "config.h"
/*
* 0x1 DSM2 less that 8 chanal 1024 Bit 
* 0x2 DSM2 8 or more chanal 1024 Bit
* 0x11/0x12 like 0x1/0x2 but 2048 Bit
*next is the chanal data, one chanal per WORD
*BITS: the lowes 10/11 bit is the chanal value, right above (4 Bit) is the chanal Number
* xxCCCCVVVVVVVVVV (10 bit) or xCCCCVVVVVVVVVVV (11Bit)*/


#define DSM_BIND_POWER 0x02
#define DSM_TX_POWER 0x07
#define DSM_RANGE_CHECK_POWER 0x03

//bits of header byte
#define SPTX_BIND_MODE (1<<7)
#define SPTX_RANGE_CHECK (1<<5)
#define SPTX_NORM_MODE (1<<4)	//00=france mode
#define SPTX_DSMX_MODE (1<<3)	//10=DSM2 11=DSMX 01=DSMX Forced

//reciver response
#define PROTOC_IS_DSMX(x)			((x&0xA0)==0xA0)
#define PROTOC_IS_DSM_11MS(x)	((x&0xB2)==0x2 ||(x&0xB2)==0xB2) 
#define PROTOC_IS_DSM_11BIT(x) (x & 0x30)>>4

#define DSM_STATE_BIND 0
#define DSM_STATE_BIND_RCV 1
#define DSM_STATE_TRANSMIT 2
#define DSM_BIND_CNT 100
#define DSM_BIND_RCV_CNT 30

#define DSM_SCAN_TIME 22000
#define DSM_TIME_CHA_CHB 4000
#define DSM_TIME11MS (11000-DSM_TIME_CHA_CHB)
#define DSM_TIME22MS (22000-DSM_TIME_CHA_CHB)
#define DSM_TIME_BIND 10000
#define DSM_SER_TO_XMIT_TIME 2000

#define DSM_BIND_DONE() (Bind_cnt==0) 

extern BYTE num_channels;	//number fo transmit channels
extern BYTE Bind_cnt;
extern BOOL DSM_Is_DSMX;		//true for DSMX false for DSM2
extern BOOL DSM_Is_11ms;		//true for 11ms update
extern BOOL DSM_Is_11Bit;		//true for 11bit resolution
extern BOOL DSM_Is_Forced;		//if true ignore reciver response
extern BOOL DSM_ready_for_Serial; 
extern WORD DSM_Perriode;		//cycle time - 4000
extern WORD Ch_Data[6];  //channel data is alwasy 11Bit resolution
extern BYTE dsm_Max_pos;	


void DSM_Init(void);
void DSM_LoadTransmitMode(void);
void DSM_StartTransmit(void);
void DSM_Cyclic(void);
void DSM_initialize_bind(void);
void DSM_build_data_packet(void);
void DSM_SetRangeCheck(BOOL val);
void DSM_NewSerialData(void);
void DSM_ClearRxError(void);

#endif //DSM_H