//*****************************************************************************
//*****************************************************************************
//  FILENAME: DIAG_OUT2.h
//   Version: 2.00, Updated on 2015/3/4 at 22:26:37                                          
//  Generated by PSoC Designer 5.4.3191
//
//  DESCRIPTION: DIAG_OUT2 User Module C Language interface file
//-----------------------------------------------------------------------------
//  Copyright (c) Cypress Semiconductor 2015. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************
#ifndef DIAG_OUT2_INCLUDE
#define DIAG_OUT2_INCLUDE

#include <m8c.h>


/* Create pragmas to support proper argument and return value passing */
#pragma fastcall16  DIAG_OUT2_Stop
#pragma fastcall16  DIAG_OUT2_Start
#pragma fastcall16  DIAG_OUT2_On
#pragma fastcall16  DIAG_OUT2_Off
#pragma fastcall16  DIAG_OUT2_Switch
#pragma fastcall16  DIAG_OUT2_Invert
#pragma fastcall16  DIAG_OUT2_GetState


//-------------------------------------------------
// Constants for DIAG_OUT2 API's.
//-------------------------------------------------
//
#define  DIAG_OUT2_ON   1
#define  DIAG_OUT2_OFF  0

//-------------------------------------------------
// Prototypes of the DIAG_OUT2 API.
//-------------------------------------------------
extern void  DIAG_OUT2_Start(void);                                     
extern void  DIAG_OUT2_Stop(void);                                      
extern void  DIAG_OUT2_On(void);                                      
extern void  DIAG_OUT2_Off(void);                                      
extern void  DIAG_OUT2_Switch(BYTE bSwitch);
extern void  DIAG_OUT2_Invert(void);                                         
extern BYTE  DIAG_OUT2_GetState(void);                                         

//-------------------------------------------------
// Define global variables.                 
//-------------------------------------------------



#endif
