/*this module contains the code for serial reciving the data from the Transmitter
* a Buffer of 16 byte ist used to store recived data
* most work is doen within the interrupt sevice routine
*datarate is set in the chip design to 115200 by the devider for VC3
*the timer module will call the ser_Tic function once per ms
*if for 5ms no more chars are recived the index is reset to zerro
*/

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "serial.h"



BYTE ser_databuf[SER_BUFFER_SIZE]; //the buffer to hold the recived data
BYTE ser_idx=0;						//index into buffer
BYTE ser_status=0;					//reciver status
BYTE ser_to=0;						//timeout variable to determin end of transmision

ser_callback_t serial_cb;

void SerialSetCallback(ser_callback_t cb)
{
	serial_cb=cb;
}

//initialize the RX8 module
void SerialRX_Init(void )
{
	int n;
	serial_cb=0;
	RX8_1_EnableInt(); //enable hardwar
	//clear buffer
	for (n=0;n<16;n++)
		ser_databuf[n]=0;
	//set index to 0
	ser_idx=0;
	//enable interrupts
	M8C_EnableGInt;
	RX8_1_EnableInt();
	//start reciving
	RX8_1_Start(RX8_PARITY_NONE);
}

//check if there is a complete package avaliable
BOOL SerialDataReady(void )
{
	//check if status contains anny error bits
	if (0xF0&SER_GET_STATUS())
	{	//reset in case of an error
		SER_CLEAR_ERROR();
		SER_RESET();
		return 0;
	}
	//ckeck if buffer is full
	if (SER_PEAK())
	{
		return 1;
	}
return 0;
}


//this is called from the system timer once per 1,953125 ms
//ser_to is reset to zerro each time a char is recived by the serial interupt service
//if there has not been anny char recived for 3 consecutive calls then reset the index
void ser_Tic(void )
{
	ser_to++; //increment timeout
	if (ser_idx!=0 && ser_to > 3) //to we have a timeout (>6ms)
		ser_idx=0; //reset index
}

#define RX8_1_RX_ENABLE 1
#pragma interrupt_handler RX8_1_Interrupt

//interrupt handler called if a char is recived
void RX8_1_Interrupt(void )
{
	static BYTE ser_c; //the char buffer, defined as static to save stack space
	
	ser_to=0; //reset timeout


	//if a char is recived 
	//we use while instead of if because it can happen that another char is recived while we are here, 
	//in that case no extra interrop for that one will occour
	while (RX8_RX_COMPLETE & RX8_1_CONTROL_REG)
	{	//get the char
		ser_c=RX8_1_RX_BUFFER_REG;
		//if we have no error store the char
		if (!(RX8_1_RX_ERROR & RX8_1_CONTROL_REG))
		{
			//if there is still room in bufffer
			if (ser_idx<SER_BUFFER_SIZE)
			{//store data
				ser_databuf[ser_idx++]=ser_c;
				//check if buffer is full now
				if (ser_idx==SER_BUFFER_SIZE)
				{
					ser_status|=SER_BUFFERFULL; //flag that we are done
					if(serial_cb)
					{
						serial_cb();
						return;
					}
				}
			}
			else 
				ser_status|=SER_BUFOVERRUN_ERROR; //flag a buffer overrun
			return ;
		}
		//if we got anny error
		if(RX8_1_RX_ERROR & RX8_1_CONTROL_REG)
		{
			ser_idx=0; //reset index , trash all we have so far
			ser_status|=RX8_1_CONTROL_REG & RX8_1_RX_ERROR; //update status
			//special case framing error
			if (RX8_1_RX_FRAMING_ERROR & RX8_1_CONTROL_REG)
			{
				RX8_1_CONTROL_REG&=~RX8_1_RX_ENABLE; //disable RX
				RX8_1_CONTROL_REG|=RX8_1_RX_ENABLE; //enable RX
			}
		}
	}
} //end of ISR