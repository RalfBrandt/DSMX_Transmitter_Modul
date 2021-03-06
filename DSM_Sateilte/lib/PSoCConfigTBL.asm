;  Generated by PSoC Designer 5.4.3191
;
; =============================================================================
; FILENAME: PSoCConfigTBL.asm
;  
; Copyright (c) Cypress Semiconductor 2013. All Rights Reserved.
;  
; NOTES:
; Do not modify this file. It is generated by PSoC Designer each time the
; generate application function is run. The values of the parameters in this
; file can be modified by changing the values of the global parameters in the
; device editor.
;  
; =============================================================================
 
include "m8c.inc"
;  Personalization tables 
export LoadConfigTBL_dsm_sateilte_Bank1
export LoadConfigTBL_dsm_sateilte_Bank0
export LoadConfigTBL_dsm_sateilte_Ordered
AREA lit(rom, rel)
LoadConfigTBL_dsm_sateilte_Bank0:
;  Instance name CS_PIN, User Module LED
;  Instance name Counter16_1, User Module Counter16
;       Instance name Counter16_1, Block Name CNTR16_LSB(DBB00)
	db		23h, 00h		;Counter16_1_CONTROL_LSB_REG(DBB00CR0)
	db		21h, ffh		;Counter16_1_PERIOD_LSB_REG(DBB00DR1)
	db		22h, 00h		;Counter16_1_COMPARE_LSB_REG(DBB00DR2)
;       Instance name Counter16_1, Block Name CNTR16_MSB(DBB01)
	db		27h, 00h		;Counter16_1_CONTROL_MSB_REG(DBB01CR0)
	db		25h, ffh		;Counter16_1_PERIOD_MSB_REG(DBB01DR1)
	db		26h, 00h		;Counter16_1_COMPARE_MSB_REG(DBB01DR2)
;  Instance name LED_1, User Module LED
;  Instance name RST, User Module LED
;  Instance name RX8_1, User Module RX8
;       Instance name RX8_1, Block Name RX8(DCB03)
	db		2fh, 00h		;RX8_1_CONTROL_REG  (DCB03CR0)
	db		2dh, 00h		;RX8_1_(DCB03DR1)
	db		2eh, 00h		;RX8_1_RX_BUFFER_REG(DCB03DR2)
;  Instance name SPIM_1, User Module SPIM
;       Instance name SPIM_1, Block Name SPIM(DCB02)
	db		2bh, 00h		;SPIM_1_CONTROL_REG  (DCB02CR0)
	db		29h, 00h		;SPIM_1_TX_BUFFER_REG(DCB02DR1)
	db		2ah, 00h		;SPIM_1_RX_BUFFER_REG(DCB02DR2)
;  Instance name SleepTimer_1, User Module SleepTimer
;  Instance name TX8SW_1, User Module TX8SW
;  Global Register values Bank 0
	db		60h, 09h		; AnalogColumnInputSelect register (AMX_IN)
	db		64h, 00h		; AnalogComparatorControl0 register (CMP_CR0)
	db		66h, 00h		; AnalogComparatorControl1 register (CMP_CR1)
	db		61h, 00h		; AnalogMuxBusConfig register (AMUXCFG)
	db		e6h, 00h		; DecimatorControl_0 register (DEC_CR0)
	db		e7h, 00h		; DecimatorControl_1 register (DEC_CR1)
	db		d6h, 00h		; I2CConfig register (I2CCFG)
	db		62h, 00h		; PWM_Control register (PWM_CR)
	db		b0h, 38h		; Row_0_InputMux register (RDI0RI)
	db		b1h, 01h		; Row_0_InputSync register (RDI0SYN)
	db		b2h, 00h		; Row_0_LogicInputAMux register (RDI0IS)
	db		b3h, 33h		; Row_0_LogicSelect_0 register (RDI0LT0)
	db		b4h, 33h		; Row_0_LogicSelect_1 register (RDI0LT1)
	db		b5h, 22h		; Row_0_OutputDrive_0 register (RDI0SRO0)
	db		b6h, 10h		; Row_0_OutputDrive_1 register (RDI0SRO1)
	db		ffh
LoadConfigTBL_dsm_sateilte_Bank1:
;  Instance name CS_PIN, User Module LED
;  Instance name Counter16_1, User Module Counter16
;       Instance name Counter16_1, Block Name CNTR16_LSB(DBB00)
	db		20h, 01h		;Counter16_1_FUNC_LSB_REG(DBB00FN)
	db		21h, 06h		;Counter16_1_INPUT_LSB_REG(DBB00IN)
	db		22h, 40h		;Counter16_1_OUTPUT_LSB_REG(DBB00OU)
;       Instance name Counter16_1, Block Name CNTR16_MSB(DBB01)
	db		24h, 21h		;Counter16_1_FUNC_MSB_REG(DBB01FN)
	db		25h, 36h		;Counter16_1_INPUT_MSB_REG(DBB01IN)
	db		26h, 44h		;Counter16_1_OUTPUT_MSB_REG(DBB01OU)
;  Instance name LED_1, User Module LED
;  Instance name RST, User Module LED
;  Instance name RX8_1, User Module RX8
;       Instance name RX8_1, Block Name RX8(DCB03)
	db		2ch, 05h		;RX8_1_FUNC_REG     (DCB03FN)
	db		2dh, e1h		;RX8_1_INPUT_REG    (DCB03IN)
	db		2eh, 40h		;RX8_1_OUTPUT_REG   (DCB03OU)
;  Instance name SPIM_1, User Module SPIM
;       Instance name SPIM_1, Block Name SPIM(DCB02)
	db		28h, 06h		;SPIM_1_FUNCTION_REG (DCB02FN)
	db		29h, c5h		;SPIM_1_INPUT_REG    (DCB02IN)
	db		2ah, 7dh		;SPIM_1_OUTPUT_REG   (DCB02OU)
;  Instance name SleepTimer_1, User Module SleepTimer
;  Instance name TX8SW_1, User Module TX8SW
;  Global Register values Bank 1
	db		61h, 00h		; AnalogClockSelect1 register (CLK_CR1)
	db		6bh, 04h		; AnalogColumnClockDivide register (CLK_CR3)
	db		60h, 00h		; AnalogColumnClockSelect register (CLK_CR0)
	db		62h, 00h		; AnalogIOControl_0 register (ABF_CR0)
	db		67h, 33h		; AnalogLUTControl0 register (ALT_CR0)
	db		64h, 00h		; ComparatorGlobalOutEn register (CMP_GO_EN)
	db		fdh, 00h		; DAC_Control register (DAC_CR)
	db		d1h, 00h		; GlobalDigitalInterconnect_Drive_Even_Input register (GDI_E_IN)
	db		d3h, 00h		; GlobalDigitalInterconnect_Drive_Even_Output register (GDI_E_OU)
	db		d0h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Input register (GDI_O_IN)
	db		d2h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Output register (GDI_O_OU)
	db		e1h, b1h		; OscillatorControl_1 register (OSC_CR1)
	db		e2h, 00h		; OscillatorControl_2 register (OSC_CR2)
	db		dfh, 19h		; OscillatorControl_3 register (OSC_CR3)
	db		deh, 00h		; OscillatorControl_4 register (OSC_CR4)
	db		ddh, 00h		; OscillatorGlobalBusEnableControl register (OSC_GO_EN)
	db		d8h, 00h		; Port_0_MUXBusCtrl register (MUX_CR0)
	db		d9h, 00h		; Port_1_MUXBusCtrl register (MUX_CR1)
	db		dah, 00h		; Port_2_MUXBusCtrl register (MUX_CR2)
	db		dbh, 00h		; Port_3_MUXBusCtrl register (MUX_CR3)
	db		ffh
AREA psoc_config(rom, rel)
LoadConfigTBL_dsm_sateilte_Ordered:
;  Ordered Global Register values
	M8C_SetBank0
	mov	reg[00h], 00h		; Port_0_Data register (PRT0DR)
	M8C_SetBank1
	mov	reg[00h], b8h		; Port_0_DriveMode_0 register (PRT0DM0)
	mov	reg[01h], 47h		; Port_0_DriveMode_1 register (PRT0DM1)
	M8C_SetBank0
	mov	reg[03h], 06h		; Port_0_DriveMode_2 register (PRT0DM2)
	mov	reg[02h], 29h		; Port_0_GlobalSelect register (PRT0GS)
	M8C_SetBank1
	mov	reg[02h], 00h		; Port_0_IntCtrl_0 register (PRT0IC0)
	mov	reg[03h], 40h		; Port_0_IntCtrl_1 register (PRT0IC1)
	M8C_SetBank0
	mov	reg[01h], 40h		; Port_0_IntEn register (PRT0IE)
	mov	reg[04h], 00h		; Port_1_Data register (PRT1DR)
	M8C_SetBank1
	mov	reg[04h], 2dh		; Port_1_DriveMode_0 register (PRT1DM0)
	mov	reg[05h], d2h		; Port_1_DriveMode_1 register (PRT1DM1)
	M8C_SetBank0
	mov	reg[07h], 82h		; Port_1_DriveMode_2 register (PRT1DM2)
	mov	reg[06h], 40h		; Port_1_GlobalSelect register (PRT1GS)
	M8C_SetBank1
	mov	reg[06h], 00h		; Port_1_IntCtrl_0 register (PRT1IC0)
	mov	reg[07h], 00h		; Port_1_IntCtrl_1 register (PRT1IC1)
	M8C_SetBank0
	mov	reg[05h], 00h		; Port_1_IntEn register (PRT1IE)
	mov	reg[08h], 00h		; Port_2_Data register (PRT2DR)
	M8C_SetBank1
	mov	reg[08h], 10h		; Port_2_DriveMode_0 register (PRT2DM0)
	mov	reg[09h], efh		; Port_2_DriveMode_1 register (PRT2DM1)
	M8C_SetBank0
	mov	reg[0bh], efh		; Port_2_DriveMode_2 register (PRT2DM2)
	mov	reg[0ah], 10h		; Port_2_GlobalSelect register (PRT2GS)
	M8C_SetBank1
	mov	reg[0ah], 00h		; Port_2_IntCtrl_0 register (PRT2IC0)
	mov	reg[0bh], 00h		; Port_2_IntCtrl_1 register (PRT2IC1)
	M8C_SetBank0
	mov	reg[09h], 00h		; Port_2_IntEn register (PRT2IE)
	mov	reg[0ch], 00h		; Port_3_Data register (PRT3DR)
	M8C_SetBank1
	mov	reg[0ch], 01h		; Port_3_DriveMode_0 register (PRT3DM0)
	mov	reg[0dh], 0eh		; Port_3_DriveMode_1 register (PRT3DM1)
	M8C_SetBank0
	mov	reg[0fh], 0eh		; Port_3_DriveMode_2 register (PRT3DM2)
	mov	reg[0eh], 00h		; Port_3_GlobalSelect register (PRT3GS)
	M8C_SetBank1
	mov	reg[0eh], 00h		; Port_3_IntCtrl_0 register (PRT3IC0)
	mov	reg[0fh], 00h		; Port_3_IntCtrl_1 register (PRT3IC1)
	M8C_SetBank0
	mov	reg[0dh], 00h		; Port_3_IntEn register (PRT3IE)
	M8C_SetBank0
	ret


; PSoC Configuration file trailer PsocConfig.asm
