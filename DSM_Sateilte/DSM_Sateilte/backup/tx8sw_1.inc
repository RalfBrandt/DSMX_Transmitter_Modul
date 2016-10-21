;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME:   TX8SW_1.inc
;;  Version: 1.30, Updated on 2015/3/4 at 22:27:54
;;  Generated by PSoC Designer 5.4.3191
;;
;;  DESCRIPTION:  Assembler declarations for the TX8SW user module interface.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress Semiconductor 2015. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "GlobalParams.inc"

TX8SW_1_Port:          equ   0x1

TX8SW_1_PortDR:        equ   PRT1DR
TX8SW_1_PortDM0:       equ   PRT1DM0
TX8SW_1_PortDM1:       equ   PRT1DM1
TX8SW_1_PortDM2:       equ   PRT1DM2

TX8SW_1_PinMask:       equ   0x1

TX8SW_1_BAUD:          equ   0x1

TX8SW_1_BAUD_115200:   equ   0x01
TX8SW_1_BAUD_57600:    equ   0x02
TX8SW_1_BAUD_38400:    equ   0x04
TX8SW_1_BAUD_19200:    equ   0x08
TX8SW_1_BAUD_9600:     equ   0x10
TX8SW_1_BAUD_4800:     equ   0x20
TX8SW_1_BAUD_2400:     equ   0x40
TX8SW_1_BAUD_1200:     equ   0x80

TX8SW_1_PARITY:        equ   0
TX8SW_1_PARITY_NONE:   equ   0
TX8SW_1_PARITY_ODD:    equ   1
TX8SW_1_PARITY_EVEN:   equ   2

TX8SW_1_DATA_BITS:     equ   2
TX8SW_1_DATA_BITS_7:   equ   1
TX8SW_1_DATA_BITS_8:   equ   2

TX8SW_1_SEND_2_STOP:   equ   0

TX8SW_1_SEND_8:        equ   (TX8SW_1_DATA_BITS & TX8SW_1_DATA_BITS_8) + TX8SW_1_PARITY
TX8SW_1_SEND_9:        equ   (TX8SW_1_DATA_BITS & TX8SW_1_DATA_BITS_8) * TX8SW_1_PARITY

TX8SW_1_CYCL_26:       equ   1
TX8SW_1_CYCL_39:       equ   2
TX8SW_1_CYCL_78:       equ   4

;------------------------------------------------------
; CPU frequency constants for  TX8SW_1
;------------------------------------------------------
TX8SW_1_SYS_CLK_24:    equ   0x01
TX8SW_1_SYS_CLK_12:    equ   0x02
TX8SW_1_SYS_CLK_6:     equ   0x04

IF (POWER_SETTING & POWER_SET_SLOW_IMO)      ; *** 6MHZ Main Oscillator ***
    TX8SW_1_SYS_CLK: equ TX8SW_1_SYS_CLK_6
ELSE                                         ; *** 24MHZ Main Oscillator ***
    TX8SW_1_SYS_CLK: equ TX8SW_1_SYS_CLK_24
ENDIF


IF (POWER_SETTING & POWER_SET_2V7_12MHZ)         ; *** 2.7 Volts / 12MHZ operation ***
    TX8SW_1_SYS_CLK: equ TX8SW_1_SYS_CLK_12
ENDIF ; *** 2.7 Volts / 12MHZ operation ***

IF (POWER_SETTING & POWER_SET_2V7_6MHZ)          ; *** 2.7 Volts /  6MHZ operation ***
    TX8SW_1_SYS_CLK: equ TX8SW_1_SYS_CLK_6
ENDIF ; *** 2.7 Volts /  6MHZ operation ***


TX8SW_1_BIT_2:         equ 4
TX8SW_1_BIT_1:         equ 2
TX8SW_1_BIT_0:         equ 1

TX8SW_1_CPU_CLK_1:     equ 3
TX8SW_1_CPU_CLK_2:     equ 2
TX8SW_1_CPU_CLK_4:     equ 1

TX8SW_1_DELAY:         equ 3
TX8SW_1_LONG_DELAY:    equ 15
TX8SW_1_MEDIUM_DELAY:  equ 11
TX8SW_1_SHORT_DELAY:   equ 5

IF (~CPU_CLOCK & TX8SW_1_BIT_2)  ; Proceed for CPU Clock > SysClk/16
    IF (CPU_CLOCK)  ; Exclude CPU Clock = SysClk/8
        IF (TX8SW_1_SYS_CLK & TX8SW_1_SYS_CLK_24) ; IMO = 24 Mhz
            IF (CPU_CLOCK & TX8SW_1_BIT_0)
                IF (CPU_CLOCK & TX8SW_1_BIT_1) ; CPU = IMO/1 = 24 Mhz
                    TX8SW_1_DELAY: equ TX8SW_1_LONG_DELAY
                ELSE
                    TX8SW_1_DELAY: equ TX8SW_1_SHORT_DELAY       ; CPU = IMO/4 = 6 Mhz       
                ENDIF                
            ELSE
                    TX8SW_1_DELAY: equ TX8SW_1_MEDIUM_DELAY       ; CPU = IMO/2 = 12 Mhz
            ENDIF
        ENDIF

        IF (TX8SW_1_SYS_CLK & TX8SW_1_SYS_CLK_12) ; IMO = 12 Mhz
            IF (CPU_CLOCK & TX8SW_1_BIT_0)
                IF (CPU_CLOCK & TX8SW_1_BIT_1) ; CPU = IMO/1 = 12 Mhz
                    TX8SW_1_DELAY: equ TX8SW_1_MEDIUM_DELAY
                ENDIF
            ELSE
                IF (CPU_CLOCK & TX8SW_1_BIT_1) ; CPU = IMO/2 = 6 Mhz
                    TX8SW_1_DELAY: equ TX8SW_1_SHORT_DELAY
                ENDIF
            ENDIF
        ENDIF
        
        IF (TX8SW_1_SYS_CLK & TX8SW_1_SYS_CLK_6) ; IMO = 6 Mhz
            IF (CPU_CLOCK & TX8SW_1_BIT_0)
                IF (CPU_CLOCK & TX8SW_1_BIT_1) ; CPU = IMO/1 = 6 Mhz
                    TX8SW_1_DELAY: equ TX8SW_1_SHORT_DELAY
                ENDIF
            ENDIF
        ENDIF
    ENDIF
ENDIF

;------------------------------------------------------
; OSC_CR0 settings depending on IMO frequency and baudrate
;------------------------------------------------------
IF (TX8SW_1_SYS_CLK & TX8SW_1_SYS_CLK_24)
  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_115200)
    TX8SW_1_OSC_CR0:  equ 0
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_26
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_57600)
    TX8SW_1_OSC_CR0:  equ 4
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_26
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_38400)
    TX8SW_1_OSC_CR0:  equ 4
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_19200)
    TX8SW_1_OSC_CR0:  equ 5
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_9600)
    TX8SW_1_OSC_CR0:  equ 5
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_78
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_4800)
    TX8SW_1_OSC_CR0:  equ 6
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_2400)
    TX8SW_1_OSC_CR0:  equ 7
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_1200)
    TX8SW_1_OSC_CR0:  equ 7
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_78
  ENDIF
ENDIF

IF (TX8SW_1_SYS_CLK & TX8SW_1_SYS_CLK_12)
  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_115200)
    TX8SW_1_OSC_CR0:  equ 1
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_26
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_57600)
    TX8SW_1_OSC_CR0:  equ 0
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_26
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_38400)
    TX8SW_1_OSC_CR0:  equ 0
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_19200)
    TX8SW_1_OSC_CR0:  equ 4
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_9600)
    TX8SW_1_OSC_CR0:  equ 5
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_4800)
    TX8SW_1_OSC_CR0:  equ 5
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_78
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_2400)
    TX8SW_1_OSC_CR0:  equ 6
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_1200)
    TX8SW_1_OSC_CR0:  equ 7
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF
ENDIF

IF (TX8SW_1_SYS_CLK & TX8SW_1_SYS_CLK_6)
  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_115200)
    TX8SW_1_OSC_CR0:  equ 2
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_26
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_57600)
    TX8SW_1_OSC_CR0:  equ 1
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_26
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_38400)
    TX8SW_1_OSC_CR0:  equ 1
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_19200)
    TX8SW_1_OSC_CR0:  equ 0
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_9600)
    TX8SW_1_OSC_CR0:  equ 4
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_4800)
    TX8SW_1_OSC_CR0:  equ 5
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_2400)
    TX8SW_1_OSC_CR0:  equ 5
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_78
  ENDIF

  IF (TX8SW_1_BAUD & TX8SW_1_BAUD_1200)
    TX8SW_1_OSC_CR0:  equ 6
    TX8SW_1_CYCL:     equ TX8SW_1_CYCL_39
  ENDIF
ENDIF

;------------------------------------------------------
;  Macros for  TX8SW_1
;------------------------------------------------------

macro TX8SW_1_Start_M
    or      [Port_1_Data_SHADE], TX8SW_1_PinMask
    mov     A, [Port_1_Data_SHADE]
    mov     reg[TX8SW_1_PortDR], A
    and     reg[TX8SW_1_PortDM2], ~TX8SW_1_PinMask
    M8C_SetBank1
    or      reg[TX8SW_1_PortDM0], TX8SW_1_PinMask
    and     reg[TX8SW_1_PortDM1], ~TX8SW_1_PinMask
    M8C_SetBank0
endm

macro TX8SW_1_Stop_M
    or      reg[TX8SW_1_PortDM2], TX8SW_1_PinMask
    M8C_SetBank1
    or      reg[TX8SW_1_PortDM1], TX8SW_1_PinMask
    and     reg[TX8SW_1_PortDM0], ~TX8SW_1_PinMask
    M8C_SetBank0
endm

macro TX8SW_1_SendBit                          ; = 26
    rrc     [TX8SW_1_bData]                    ; 7
    jc      . + 6                                       ; 5
    and     A, ~TX8SW_1_PinMask                ; 4
    jmp     . + 6                                       ; 5
    or      A, TX8SW_1_PinMask                 ; 4
    jmp     . + 2                                       ; 5
    mov     reg[TX8SW_1_PortDR], A             ; 5
endm


; end of file TX8SW_1.inc