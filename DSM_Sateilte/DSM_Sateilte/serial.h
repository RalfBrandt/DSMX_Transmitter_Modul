/*this module contains the code for serial reciving the data from the sattilie reciver
* a Buffer of 16 byte ist used to store recived data
* most work is doen within the interrupt sevice routine
*datarate is set in the chip design to 115200 by the devider for VC3
*the timer module will call the ser_Tic function once per ms
*if for 5ms no more chars are recived the index is reset to zerro
*/
//prevent double include
#ifndef SERIAL_DEFINED_6546te54365476576575
#define SERIAL_DEFINED_6546te54365476576575

//defines
#define SER_BUFFER_SIZE 14 //the size of the recive buffer

//the meaning of the BITs of the status word
#define SER_BUFFERFULL 0x1  
#define SER_BUFOVERRUN_ERROR 	0x10
#define  SER_PARITY_ERROR		0x80
#define  SER_OVERRUN_ERROR 		0x40
#define  SER_FRAMING_ERROR 		0x20


#define SER_GET_STATUS() (ser_status) //get reciver status
#define SER_RESET_STATUS() {ser_status=0;} //guess what
#define SER_CLEAR_ERROR() {ser_status&=~0xF;}
#define SER_GET_BUFFER_CNT() (ser_idx) //return number of char recived
#define SER_RESET_BUFFER()  {ser_idx=0;}
#define SER_RESET() {SER_RESET_STATUS();SER_RESET_BUFFER();} //reset status und buffer index

#define SER_PEAK() (SER_BUFFERFULL&SER_GET_STATUS()) //check if buffer is full

//data
extern BYTE ser_databuf[SER_BUFFER_SIZE]; //the buffer to hold recived data
extern BYTE ser_idx;					//index into buffer
extern BYTE ser_status;					//current recive status
extern BYTE ser_to;						//used for timeout determination

typedef void (*ser_callback_t)(void);
//prototypes
void SerialRX_Init(void);			//initialize ther serial reciver
BOOL SerialDataReady(void );			//returns true if recive buffer is full (data avaliabe to be proceed
void ser_Tic(void );				//called by systimer one per ms to determin timeout
void SerialConnectToPin(void);		//reconnect RX8 to input pin
void SerialSetCallback(ser_callback_t cb);
#endif 