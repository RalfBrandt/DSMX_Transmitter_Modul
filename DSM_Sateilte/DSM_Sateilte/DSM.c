#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "dsm.h"
#include "serial.h"
#include "timer.h"

extern BYTE ModellNr;


//share memory with flash buffer
extern RAM_D mem;
#define tx_buf mem.b.buf0
//BYTE tx_buf[16];	//transmit buffer
#define bind_buf mem.b.buf0
//BYTE bind_buf[16];	//transmit buffer
#define rx_buf mem.b.buf1
//BYTE rx_buf[16];	//recive buffer

//config data
BOOL DSM_Is_DSMX;		//true for DSMX false for DSM2
BOOL DSM_Is_11ms;		//true for 11ms update
BOOL DSM_Is_11Bit;		//true for 11bit resolution

//global data
BYTE channels[23]; 	//list of channels
BYTE chidx;			//current channel index	
BOOL DSM_Is_Forced;		//if true ignore reciver response
BOOL RX_Response;	//true if we got a response from reciver
BOOL Ch_A;			//true for channel A false for channel B
BYTE dsm_Max_pos;	
BYTE sop_col;		
BYTE data_col;		
BYTE Bind_cnt;		//bind pulse counter
BYTE State;			
BYTE bind_channal;	//channel used to send bind packes
BYTE num_channels;	//number fo transmit channels
BYTE cyrfmfg_id[6];	//manufacturer ID of cyrf6936
//BYTE cyrfmfg_id[6]={0x67,0x3B,0x16,0xCC,0xD8,0xFF};	//Does Work
//BYTE cyrfmfg_id[6]={0x76,0x65,0x3B,0x0F,0x5D,0xFC}; //Does Not Work
//BYTE cyrfmfg_id[6]={0x67,0x65,0x3B,0x0F,0x5D,0xFC}; //Test OK n
//BYTE cyrfmfg_id[6]={0x76,0x3B,0x3B,0x0F,0x5D,0xFC}; //Test OK
//BYTE cyrfmfg_id[6]={0x76,0x65,0x16,0x0F,0x5D,0xFC}; //Test OK
BYTE cyrfmfg_id[6]={0x76,0x65,0x3B,0xCC,0x5D,0xFC}; ///Test OK
BYTE tx_cfg;		//xmit power precomputed
WORD DSM_Perriode;		//cycle time - 4000
BOOL DSM_ready_for_Serial; //true if we are ready for the next serial package
int Last_Phase_error;


static const BYTE pn_bind[] = {0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84}; //used for reciving bind response
static const BYTE pn_check[] = {0xc6,0x94,0x22,0xfe,0x48,0xe6,0x57,0x4e };		//used for scanning channel RSSI (found on spi capture of LPDSM)

static const BYTE pncodes[5][9][8] = {
    /* Note these are in order transmitted (LSB 1st) */
{ /* Row 0 */
  /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
  /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
  /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
  /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
  /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
  /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
  /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
  /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
  /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
},
{ /* Row 1 */
  /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
  /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
  /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
  /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
  /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
  /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
  /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
  /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
  /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
},
{ /* Row 2 */
  /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
  /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
  /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
  /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
  /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
  /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
  /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
  /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
  /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
},
{ /* Row 3 */
  /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
  /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
  /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
  /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
  /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
  /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
  /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
  /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
  /* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
},
{ /* Row 4 */
  /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
  /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
  /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
  /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
  /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
  /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
  /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
  /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
  /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
},};


static const BYTE init_vals[][2] = {
//	{CYRF_MODE_OVERRIDE, 0x01},
    {CYRF_CLK_EN, CLK_EN_RXF},  //0x02
    {CYRF_AUTO_CAL_TIME, 0x3c}, //see cyrf manual
    {CYRF_AUTO_CAL_OFFSET, 0x14}, //see cyrf manual
	{CYRF_IO_CFG,IO_CFG_IRQ_POL}, //0x40 IRQ active High 
    {CYRF_RX_CFG, RX_LNA|FAST_TURN_EN}, //0x48 Fast Turn Mode Enable, Low Noise Amplifier ON
    {CYRF_TX_OFFSET_LSB, 0x55},//see cyrf manual
    {CYRF_TX_OFFSET_MSB, 0x05},//see cyrf manual
    {CYRF_XACT_CFG,CYRF_XACT_END_STATE(CYRF_MODE_IDLE)|CYRF_FRC_END}, //0x24 Force  Idle Mode
    //{CYRF_TX_CFG,TX_DM(TX_DM_SDR)|TX_DC_LEN}, //0x38 SDR Mode  64 chip
    {CYRF_DATA64_THOLD, 0x0a},//see cyrf manual
    {CYRF_XACT_CFG,CYRF_XACT_END_STATE(CYRF_MODE_IDLE)}, //0x04 Transaction End State Idle Mode
    //{CYRF_XTAL_CTRL,0x80},
	{CYRF_ANALOG_CTRL, ALL_SLOW}, //0x01 all slow
    {CYRF_XACT_CFG,CYRF_XACT_END_STATE(CYRF_MODE_IDLE)| CYRF_FRC_END}, //0x24 Force IDLE
	{CYRF_RX_ABORT, 0x00}, //Clear RX abort
    {CYRF_DATA64_THOLD, 0x0a}, //set pn correlation threshold
    {CYRF_FRAMING_CFG, SOP_LEN|0xa}, //set sop len and threshold
    {CYRF_RX_ABORT, 0x0f}, //Clear RX abort?
};

static const BYTE Transmit_vals[][2] = {
    //{CYRF_TX_CFG,TX_DM(TX_DM_8DR)|TX_PA_P4DBM}, //0x0F 8DR Mode  +4 dBm 32 chip codes
	{CYRF_FRAMING_CFG,LEN_EN|SOP_LEN|SOP_EN|0xA},  //0xEA SOP Enable SOP code length is 64 chips Packet Length Enable  SOP Correlator Threshold=0xA
    {CYRF_TX_OVERRIDE, 0x00},
    {CYRF_RX_OVERRIDE, 0x00},
};

static const BYTE bind_vals[][2] = {
    {CYRF_TX_CFG,TX_DC_LEN|TX_DM(TX_DM_SDR)|DSM_BIND_POWER}, //0x38 Set 64 chip, SDR mode
    {CYRF_FRAMING_CFG,SOP_LEN|0xA}, //0x4A set sop len  32 chip and threshold =0xA 
    {CYRF_TX_OVERRIDE, DIS_TXCRC}, //0x4 disable tx CRC
    {CYRF_RX_OVERRIDE, DIS_RXCRC}, //0x4 disable rx CRC
    {CYRF_EOP_CTRL, 0x02}, //set EOP sync == 2
};


static const BYTE preamble[]={0x04,0x33,0x33};
static  BYTE tx_cmd[]={0x10,TXE_IRQEN|TXC_IRQEN|TX_GO|TX_CLR}; //0xC3

#define TX_CFG (TX_DM(TX_DM_8DR)|TX_DC_LEN) //0x28 DDR Mode  64 chip codes

void DSM_SetRangeCheck(BOOL val)
{
	tx_cfg=val?TX_CFG|DSM_RANGE_CHECK_POWER:TX_CFG|DSM_TX_POWER;	
}

//set time till periode
void SetTimer(void)
{
	if(Ch_A)
	{
		//if CH_A is on, CH_B has been send out, 
		//from now till CH_A transmit is the windows where the serial data must come in  
		DSM_ready_for_Serial=TRUE;
		//DSM_Data_To_send=FALSE;
		Timer_SetPeriod(DSM_TIME_CHA_CHB);
	}
	else
	{
		Timer_SetPeriod(DSM_Perriode);
	}
}

//calculate and set channel CRC , SOP and data code 
void SetCH_CRC_SOP_DATA(void)
{
	BYTE channel=channels[chidx];
	BYTE pn_row = DSM_Is_DSMX ? (channel - 2)%5 : channel%5;
	WORD CRCSeed=Ch_A?~((WORD)(cyrfmfg_id[0] << 8) + (WORD)cyrfmfg_id[1]):((WORD)(cyrfmfg_id[0] << 8) + (WORD)cyrfmfg_id[1]);
	CYRF_SetRFChannel(channel);
	CYRF_SetCRCSeed(CRCSeed);
    CYRF_SetSOPCode(pncodes[pn_row][sop_col]);
    CYRF_SetDataCode(pncodes[pn_row][data_col], 16);
    if(DSM_Is_DSMX)
        chidx = (chidx + 1) % 23;
    else
        chidx = (chidx + 1) % 2;
}

//load the given config into cyfr6936
void DSM_Load_Config(const BYTE vals[][2],BYTE cnt)
{
BYTE n;

for(n=0;n<cnt;n++)
	{
	CYRF_WriteRegister(vals[n][0],vals[n][1]);
	}
}

//called from timer
//send out next data package
void Bind_Xmit_cb(void)
{
#ifdef DEBUG
	 //TX8SW_1_CPutString("X");
#endif
	CYRF_WriteRegisterInc(CYRF_TX_LENGTH,tx_cmd,sizeof(tx_cmd)/sizeof(BYTE));
	CYRF_WriteTx_Data(bind_buf);
}


void Send_Xmit_cb(void)
{
#ifdef DEBUG
	//TX8SW_1_CPutString("X");
	//DIAG_OUT1_Switch(Ch_A);
#endif
	//if(Ch_A) //found on SPI capture from mlp4dsm
	//	CYRF_ReadRegister(CYRF_XTAL_CTRL);
	CYRF_WriteRegisterInc(CYRF_TX_LENGTH,tx_cmd,sizeof(tx_cmd)/sizeof(BYTE));
	CYRF_WriteTx_Data(tx_buf);
	DSM_ready_for_Serial=FALSE;		
	Ch_A=!Ch_A;
}



//start reciving
void DSM_StartReceive(void)
{
	CYRF_StartReceive();
	CYRF_ReadRSSI();
}

//initialize bind state
void DSM_initialize_bind(void)
{	
	BYTE n;
	Bind_cnt=DSM_BIND_CNT;
	State=DSM_STATE_BIND;
	DSM_Load_Config(bind_vals,sizeof(bind_vals)/(sizeof(BYTE)*2));
	CYRF_SetRFChannel(bind_channal);
	CYRF_SetDataCode(pncodes[0][8], 16);

	//manual inline
	//DSM_build_bind_packet();
	//void DSM_build_bind_packet(void)
	{
	    //BYTE n;
	    WORD sum = 384 - 0x10;
	    bind_buf[0] = ~cyrfmfg_id[0];
	    bind_buf[1] = ~cyrfmfg_id[1];
	    bind_buf[2] = ~cyrfmfg_id[2];
	    bind_buf[3] = ~(cyrfmfg_id[3]+ModellNr);
	    bind_buf[4] = bind_buf[0];
	    bind_buf[5] = bind_buf[1];
	    bind_buf[6] = bind_buf[2];
	    bind_buf[7] = bind_buf[3];
		
		//gennerate first checksum
	    for(n = 0; n < 8; n++)
	        sum += bind_buf[n];
	    bind_buf[8] = sum >> 8;
	    bind_buf[9] = sum & 0xff;

		bind_buf[10] = 0x01; //?Air 0x2; //surface 
	    bind_buf[11] =num_channels;
	    bind_buf[12] = DSM_Is_DSMX?(DSM_Is_11ms?0xB2:0xA2):DSM_Is_11Bit?(DSM_Is_11ms?0x12:0x11):(DSM_Is_11ms?0x02:0x01);

#ifdef DEBUG
		TX8SW_1_PutSHexByte(bind_buf[12]);	
#endif
		
		bind_buf[13] = 0x00; //???
		
	    //gennerate second checksum 
		for(n = 8; n < 14; n++)
	        sum += bind_buf[n];
	    
		bind_buf[14] = sum >> 8;
	    bind_buf[15] = sum & 0xff;
	}
	Timer_SetPeriod(DSM_TIME_BIND);
	Timer_SetCallback(Bind_Xmit_cb);
	Timer_Start();
}


//build the data package
void DSM_build_data_packet(void)
{
BYTE i;
	//init header
   if (DSM_Is_DSMX) {
        tx_buf[0] = cyrfmfg_id[2];
        tx_buf[1] = (cyrfmfg_id[3]+ModellNr);
    } else {
        tx_buf[0] = ~cyrfmfg_id[2];
        tx_buf[1] = ~(cyrfmfg_id[3]+ModellNr);
    }
	//fill data 
    for (i = 0; i < 6; i++) 
		{
		WORD val=Ch_Data[i];
		//input data is allways 11Bit
		//so we need to shift down if we transmit DSM2 10Bit format
		if(!DSM_Is_11Bit)
			val=val>>1;
		tx_buf[i*2+2]=val>>8;
		tx_buf[i*2+3]=val&0xFF;
 		}
	tx_buf[14]=0xFF;
	tx_buf[15]=0xFF;
 }

//clear recive error and pending interupt flags
void DSM_ClearRxError(void)
{
	BYTE i=100;
	CYRF_ReadRegister(CYRF_RX_IRQ_STATUS);
	CYRF_ReadRegister(CYRF_RX_IRQ_STATUS);
	CYRF_WriteRegister(CYRF_RX_ABORT,ABORT_EN); //abort
	//force end state
	CYRF_WriteRegister(CYRF_XACT_CFG,CYRF_XACT_END_STATE(CYRF_MODE_IDLE)|CYRF_FRC_END);
    while (! (CYRF_ReadRegister(CYRF_XACT_CFG) & CYRF_XACT_END_STATE(CYRF_MODE_IDLE))) 
	{
        if(!--i)
            break;
	}
	//clear abort
	CYRF_WriteRegister(CYRF_RX_ABORT,0);
}


//called from timer
//see if we recived response from reciver
void Rcv_cb(void)
{
#ifdef DEBUG
	//TX8SW_1_CPutString("R");
#endif
	Bind_cnt--;
	//check if time is over
	if(!Bind_cnt)
	{
		Timer_Stop();
		//if we got no response abbort RX and clear anny rx error
		if(!RX_Response)
		{
			DSM_ClearRxError();
			//fallback to DSM2 10Bit 22ms if not forced 
			if(!DSM_Is_Forced)
			{
				DSM_Is_DSMX=FALSE;
				DSM_Is_11Bit=FALSE;
				DSM_Is_11ms=FALSE;
			}
		}
	State=DSM_STATE_TRANSMIT;
#ifdef DEBUG
	/*if(DSM_Is_DSMX)
		TX8SW_1_CPutString("DSMX ");
	else
		TX8SW_1_CPutString("DSM2 ");
	if(DSM_Is_11ms)
		TX8SW_1_CPutString("11ms ");
	else
		TX8SW_1_CPutString("22ms ");
	if(DSM_Is_11Bit)
		TX8SW_1_CPutString("11Bit ");
	else
		TX8SW_1_CPutString("10Bit ");*/
#endif
	}
}

//start waiting for the bind recive signal
void DSM_StartBindRcv(void)
{
	State=DSM_STATE_BIND_RCV;
	Timer_SetCallback(Rcv_cb);
    CYRF_SetDataCode(pn_bind, 16);
	Bind_cnt=DSM_BIND_RCV_CNT;
	CYRF_StartReceive();
	CYRF_ReadRSSI();
}



void DSM_Cyclic(void)
{
	if(CY_irq) //we got an IRQ from CYRF6936
	{
		BYTE Irq_Status[2];
		CY_irq=FALSE;
#ifdef DEBUG
		//DIAG_OUT1_Invert();
		//DIAG_OUT1_Invert();
		//TX8SW_1_CPutString("I");
#endif
		
		switch(State)
		{
			case DSM_STATE_BIND:
			{
				//read IRQ status
				CYRF_ReadRegisterMulti(CYRF_TX_IRQ_STATUS,Irq_Status,2);
				Irq_Status[0]|=Irq_Status[1]&TXE_IRQ;
				//TX done
				if(Irq_Status[0]&TXC_IRQ)
					{
						Bind_cnt--;
						//check if we are done (time is over)
						if(!Bind_cnt)
						{
							//start reciving the response
							DSM_StartBindRcv();
						}
					}	
			}
			return;
			case DSM_STATE_BIND_RCV:
				{
					//read IRQ status
					CYRF_ReadRegisterMulti(CYRF_RX_IRQ_STATUS,Irq_Status,2);
					Irq_Status[0]|=Irq_Status[1]&RXE_IRQ;
					if(Irq_Status[0]&RXC_IRQ)
					{
						//we got some thing
						BYTE i;
				        WORD ckSum = 0x170;
						
						BYTE rx_cnt=CYRF_Rx_cnt();
#ifdef DEBUG
						//TX8SW_1_CPutString("i");
#endif
						//read it in
						CYRF_ReadRx_data(rx_buf,rx_cnt);
						//check recived data
						if(rx_cnt!=10)
						{
							goto rcv_error;
						}
#ifdef DEBUG
						//TX8SW_1_CPutString("a");
#endif
						if(rx_buf[0]!=bind_buf[0] || rx_buf[1]!=bind_buf[1] || rx_buf[2]!=bind_buf[2] || rx_buf[3]!=bind_buf[3])
						{
							goto rcv_error;
						}
#ifdef DEBUG
						//TX8SW_1_CPutString("b");
#endif
						for (i = 0; i < 8; i++)
					         ckSum += rx_buf[i];
						if(rx_buf[8] != (ckSum >> 8) || rx_buf[9] !=(ckSum & 0xff))
						{
							goto rcv_error;
						}
#ifdef DEBUG
						//TX8SW_1_CPutString("c");
#endif
						//I have no Ida what to do with the channel count the reciver send us
						//dsmNumChannels=packet[5];
						
						//if transmitter is not forced determin operation mode
						if(!DSM_Is_Forced)
							{
							DSM_Is_DSMX=PROTOC_IS_DSMX(rx_buf[6]);
							DSM_Is_11Bit = PROTOC_IS_DSM_11BIT(rx_buf[6]);
							DSM_Is_11ms=PROTOC_IS_DSM_11MS(rx_buf[6]);
							}
						RX_Response=TRUE;
#ifdef DEBUG
						TX8SW_1_PutSHexByte(rx_buf[6]);	
#endif
					}			
				return;			
				}
				rcv_error:
					DSM_ClearRxError();
					DSM_StartBindRcv();
				return;
			default:
				{ //transmit Mode
				//get IRQ Status
				CYRF_ReadRegisterMulti(CYRF_TX_IRQ_STATUS,Irq_Status,2);
				Irq_Status[0]|=Irq_Status[1]&TXE_IRQ;
				//tx done
				if(Irq_Status[0]&TXC_IRQ)
					{
					//tx done
						//set timer for next transmit
						SetTimer();
#ifdef DEBUG
						//TX8SW_1_CPutString("C");
#endif
						//calculate and set channel SOP , CRC and data code
						SetCH_CRC_SOP_DATA();
						if(!Ch_A)
						{
							CYRF_WriteRegister(CYRF_TX_CFG,tx_cfg); //update TX power 
						}
					}	
				}
		}
	}
}


//called if there is new serial data
void DSM_NewSerialData(void)
{
	int Phase_Error;	
	int Freq_Error;
		//if we are in the windows for reciving data proceed
		if(DSM_ready_for_Serial)
		{
			//get time left till next transmit, shall be 2ms
			WORD time=Counter16_1_wReadCounter();
			//we need at least one ms for building the package
			if(time>DSM_TIME11MS && !DSM_Is_11ms )//&& ((Ch_Data[dsm_Max_pos]>>11)+1)==num_channels)
				{ //ignore extra data (transmitter running 11ms while reciver expect 11ms)
#ifdef DEBUG
				TX8SW_1_CPutString("Q");
#endif
				return;
				}
			if(time>200)
			{
				DSM_build_data_packet();
#ifdef DEBUG
				TX8SW_1_CPutString("P");
#endif
			}
#ifdef DEBUG
				TX8SW_1_CPutString("S");
#endif
		//calculate pahse error	
		Phase_Error=DSM_SER_TO_XMIT_TIME-time;
		//calculate frequency error
		//instead off measuring the perriode and the transmision perriode
		//wee use the change the Phase Error to calculate the frequency error
		Freq_Error=Phase_Error-Last_Phase_error;
		//save corrent error as last
		Last_Phase_error=Phase_Error;
		//simple double P-Term loop.
		DSM_Perriode+=Freq_Error>>3;
		DSM_Perriode+=Phase_Error>>6;
#ifdef DEBUG
			if(Phase_Error>200)
				TX8SW_1_CPutString("p");
			if(Phase_Error<-300)
				TX8SW_1_CPutString("m");
#endif
		
		}
		else //lost sync
		{
		//serial data came in at a time we did not expected it
		//porbably we are out of sync
		//resync
#ifdef DEBUG
			TX8SW_1_CPutString("L");
			//DIAG_OUT1_Switch(0);
#endif
			//manual inline
			//DSM_StartTransmit();
			//void DSM_StartTransmit(void)
			{
				State=DSM_STATE_TRANSMIT;
				Timer_SetCallback(Send_Xmit_cb);
				//to set timer current value we need to
				//stop timer ans set period instead
				Timer_Stop();
				Timer_SetPeriod(DSM_SER_TO_XMIT_TIME);
				//start timer
				Timer_Start();
				Last_Phase_error=0;
				Ch_A=TRUE;
				SetTimer();
			}
		}
		DSM_build_data_packet();
}

//calculate dsmX channels 
//found on deviation-tx source
// math by Alexandr Alexandrov code by Sergey Gimaev
//pseudo random gennerator /Linear congruential generator based on "Numerical Recipes" Chapter 7.1
void calc_dsmx_channel(void)
{
    BYTE idx; 
	DWORD id_tmp;
    DWORD id = ~(((DWORD)cyrfmfg_id[0] << 24) | ((DWORD)cyrfmfg_id[1] << 16) | ((DWORD)cyrfmfg_id[2] << 8) | (((DWORD)cyrfmfg_id[3]+ModellNr) << 0));
	idx = 0;
    id_tmp = id;
    while(idx < 23) {
        int i;
        BYTE next_ch;
		int count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;
        id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F; // Randomization
        next_ch = ((id_tmp >> 8) % 0x49) + 3;       // Use least-significant byte and must be larger than 3
        if (((next_ch ^ id) & 0x01 )== 0)
            continue;
        for (i = 0; i < idx; i++) {
            if(channels[i] == next_ch)
                break;
            if(channels[i] <= 27)
                count_3_27++;
            else if (channels[i] <= 51)
                count_28_51++;
            else
                count_52_76++;
        }
        if (i != idx)
            continue;
        if ((next_ch < 28 && count_3_27 < 8)
          ||(next_ch >= 28 && next_ch < 52 && count_28_51 < 7)
          ||(next_ch >= 52 && count_52_76 < 8))
        {
            channels[idx++] = next_ch;
#ifdef DEBUG
		TX8SW_1_PutSHexByte(next_ch);	
#endif
        }
    }
}

BOOL cb_tick;
//called form timer
//set flag that timer ticked
void src_cb(void)
{
	cb_tick=TRUE;
	#ifdef DEBUG
	//TX8SW_1_CPutString("t");
	#endif

}

//find 2 channels with low noise
void find_dsm2_channel(void)
{
BYTE ch;
#ifdef DEBUG
	//TX8SW_1_CPutString("F");
#endif
	//set timing	
	Timer_SetCallback(src_cb);
	Timer_SetPeriod(DSM_SCAN_TIME);
	Timer_Start();
	cb_tick=FALSE;
	//loop thru both channels
	for(ch=0;ch<2;)
	{
		BYTE rssi=0;
		//take from spi capture of MLP4DSM
		CYRF_WriteRegister(CYRF_DATA64_THOLD,0x3F);
		CYRF_WriteRegister(CYRF_FRAMING_CFG,0xFF);
		CYRF_SetSOPCode(pn_check);
		//loop thru avaliable channels
		bind_channal+=5;
		bind_channal%=0x4F;
		bind_channal|=1;
		CYRF_SetRFChannel(bind_channal);
		//start reciving
		DSM_StartReceive();
		//capture max of rssi
		while(!cb_tick)
		{
			BYTE val=0x1F&CYRF_ReadRSSI();
			if(val>rssi)
				rssi=val;
		}
#ifdef DEBUG
		//TX8SW_1_CPutString("T");
		//TX8SW_1_PutSHexByte(bind_channal);
#endif
		cb_tick=FALSE;
		//low noise? use this one
		if(rssi<15)
			{
			channels[ch]=bind_channal;
			ch++;
#ifdef DEBUG
			//TX8SW_1_CPutString("P ");
#endif
			}
	//read away any data we recived and clear any error
	CYRF_ReadRx_data(rx_buf,CYRF_Rx_cnt());
	DSM_ClearRxError();
	}
	//we are done stop timer
	Timer_SetCallback(0);
	Timer_Stop();
#ifdef DEBUG
		//TX8SW_1_CPutString("E");
#endif
}
//coad configuration for transmit mode
void DSM_LoadTransmitMode(void)
{
	CYRF_WriteRegister(CYRF_TX_CFG,tx_cfg); 
	DSM_Load_Config(Transmit_vals,sizeof(Transmit_vals)/(sizeof(BYTE)*2));
	if(DSM_Is_DSMX)
		{
#ifdef DEBUG
			//TX8SW_1_CPutString("CH_");
#endif
		calc_dsmx_channel();
		}
	else
		{
		find_dsm2_channel();
		}	
	//set periode to be 11ms or 22ms	
	DSM_Perriode=DSM_Is_11ms?DSM_TIME11MS:DSM_TIME22MS;
	//calculate channel CRC SOP and data code for first channel to transmitt
	SetCH_CRC_SOP_DATA();
	
#ifdef DEBUG
	if(DSM_Perriode==DSM_TIME11MS)
		TX8SW_1_CPutString("T11");
	if(DSM_Perriode==DSM_TIME22MS)
		TX8SW_1_CPutString("T22");
#endif
}


//initialise system
void DSM_Init(void)
{
	BYTE n;
	//init CYRF6936
	CYRF_Init();
	//set defalut values
	DSM_Is_DSMX=TRUE;
	DSM_Is_11ms=TRUE;
	DSM_Is_11Bit=TRUE;
	DSM_Is_Forced=FALSE;
	RX_Response=FALSE;
	Ch_A=TRUE;
	chidx=0;
	DSM_ready_for_Serial=FALSE;
	Bind_cnt=0;
	num_channels=6;
	//set tx power to defalut
	tx_cfg=TX_CFG|DSM_TX_POWER;
	
	CYRF_WritePreamble(preamble);
	CYRF_GetMfgData(cyrfmfg_id);
#ifdef SHIFT_MFG_ID
	for (n=0;n<6;n++)
		{
		cyrfmfg_id[n]=cyrfmfg_id[(n+1)%6];
		}
#endif
#ifdef DEBUG
	TX8SW_1_CPutString("MfgId:");
	for (n=0;n<6;n++)
		{
		TX8SW_1_PutSHexByte(cyrfmfg_id[n]);	
		}
#endif
	//calculate sop and data column
	sop_col = (cyrfmfg_id[0] + cyrfmfg_id[1] + cyrfmfg_id[2] + 2) & 0x07;
    data_col = 7 - sop_col;
	CYRF_SetRFChannel(0x61);
	//get some random
	//for choosing a random bind cannel we need some randomness.
	//but on a microcontroller this is hard to archive
	//so use some noise we capture on channel 0x61
	//if you look on the SPI trace from MLP4DSM spectrum seems to do something simmular
	DSM_StartReceive();
	CYRF_WriteRegister(CYRF_DATA64_THOLD,0x3F);
	CYRF_WriteRegister(CYRF_FRAMING_CFG,0x7F);
	DSM_StartReceive();
	Timer_Wait_ms(10);
	CYRF_ReadRx_data(rx_buf,CYRF_Rx_cnt());
	DSM_ClearRxError();
	DSM_Load_Config(init_vals,sizeof(init_vals)/(sizeof(BYTE)*2));

#ifdef DEBUG
	TX8SW_1_CPutString(" BindCH:");
#endif
	//simpel xor sum of what we got
	for(n=0;n<16;n++)
		bind_channal^=rx_buf[n];
	//add systic
	bind_channal+=(systic&0xFF)^(systic>>8);
	bind_channal&=0x3F;
	bind_channal|=1; //make shure channel is odd
#ifdef DEBUG
	TX8SW_1_PutSHexByte(bind_channal);
#endif
	CY_irq=FALSE;;
}
