Project:      output/DSM_Sateilte
Build Number: 0
Date:         Fri Oct 21 20:20:42 2016
Compiler:     ICCM8C
Version:      7.05.00


Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   0170   01AB =    427. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       0170  dsm.c:pn_bind
       0178  dsm.c:pn_check
       0180  dsm.c:pncodes
       02E8  dsm.c:init_vals
       0308  dsm.c:Transmit_vals
       030E  dsm.c:bind_vals
       0318  dsm.c:preamble
       031B  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   031B   000F =     15. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       031B  __idata_start
       032A  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                        func_lit   032A   0010 =     16. bytes (rel,con,rom,proclab)

       Addr  Global Symbol
      -----  --------------------------------
       032A  __func_lit_start
       033A  __func_lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     psoc_config   033A   014F =    335. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       033A  LoadConfigTBL_dsm_sateilte
       0476  LoadConfigInit
       0476  _LoadConfigInit
       0483  _LoadConfig_dsm_sateilte
       0483  LoadConfig_dsm_sateilte
       0489  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   0489   04FC =   1276. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0489  _SPIM_1_EnableInt
       0489  _SPIM_1_Start
       0489  SPIM_1_Start
       0489  _SPIM_1_DisableInt
       048E  _SPIM_1_SendTxData
       048E  _SPIM_1_Stop
       048E  SPIM_1_SendTxData
       0491  _SPIM_1_bReadRxData
       0491  _bSPIM_1_ReadRxData
       0491  SPIM_1_bReadRxData
       0491  bSPIM_1_ReadRxData
       0494  _SPIM_1_bReadStatus
       0494  _bSPIM_1_ReadStatus
       0494  SPIM_1_bReadStatus
       0494  bSPIM_1_ReadStatus
       0497  _SleepTimer_1_ISR
       04B2  _SleepTimer_1_EnableInt
       04B2  SleepTimer_1_EnableInt
       04B6  _SleepTimer_1_DisableInt
       04B6  SleepTimer_1_Start
       04B6  _SleepTimer_1_Start
       04C3  _SleepTimer_1_SetInterval
       04C3  _SleepTimer_1_Stop
       04C3  SleepTimer_1_TickWait
       04C3  _SleepTimer_1_TickWait
       04D2  _SleepTimer_1_iGetTickCntr
       04D2  _SleepTimer_1_bGetTimer
       04D2  _SleepTimer_1_SetTimer
       04D2  _SleepTimer_1_SyncWait
       04D2  _RX8_1_ISR
       04D2  _SleepTimer_1_bGetTickCntr
       04D6  _RX8_1_EnableInt
       04D6  RX8_1_EnableInt
       04DA  _RX8_1_Start
       04DA  RX8_1_Start
       04DA  _RX8_1_DisableInt
       04DF  bRX8_1_ReadRxData
       04DF  _RX8_1_cGetChar
       04DF  bRX8_1_ReadRxStatus
       04DF  _bRX8_1_ReadRxStatus
       04DF  _RX8_1_Stop
       04DF  RX8_1_cGetChar
       04DF  _RX8_1_bReadRxData
       04DF  _RX8_1_bReadRxStatus
       04DF  _bRX8_1_ReadRxData
       04E7  _RST_On
       04E7  _RX8_1_cReadChar
       04E7  _RX8_1_iReadChar
       04E7  RST_On
       04EB  _RST_Stop
       04EB  _RST_Start
       04EB  RST_Start
       04EB  _RST_Off
       04EB  RST_Stop
       04EB  RST_Off
       04ED  RST_Switch
       04ED  _RST_Switch
       04FE  _LED_1_Off
       04FE  _LED_1_Start
       04FE  LED_1_Start
       04FE  RST_GetState
       04FE  LED_1_On
       04FE  _LED_1_Stop
       04FE  RST_Invert
       04FE  LED_1_Stop
       04FE  LED_1_Off
       0500  LED_1_Switch
       0500  _LED_1_Switch
       0511  LED_1_Invert
       0511  _LED_1_Invert
       0519  E2PROM_1_Start
       0519  _E2PROM_1_Start
       0519  LED_1_GetState
       051A  E2PROM_1_bE2Write
       051A  _E2PROM_1_Stop
       051A  _E2PROM_1_bE2Write
       0526  E2PROM_1_E2Read
       0526  _E2PROM_1_E2Read
       0531  CS_PIN_On
       0531  _CS_PIN_On
       0535  CS_PIN_Start
       0535  _CS_PIN_Off
       0535  CS_PIN_Stop
       0535  _CS_PIN_Stop
       0535  _CS_PIN_Start
       0535  CS_PIN_Off
       0537  CS_PIN_Switch
       0537  _CS_PIN_Switch
       0548  CS_PIN_GetState
       0548  _Counter16_1_ISR
       0548  CS_PIN_Invert
       054C  _Counter16_1_EnableInt
       054C  Counter16_1_EnableInt
       0550  _Counter16_1_DisableInt
       0550  _Counter16_1_Start
       0550  Counter16_1_Start
       0554  Counter16_1_Stop
       0554  _Counter16_1_Stop
       0558  _Counter16_1_WritePeriod
       0558  Counter16_1_WritePeriod
       055E  _Counter16_1_wReadCounter
       055E  _wCounter16_1_ReadCounter
       055E  Counter16_1_wReadCounter
       055E  wCounter16_1_ReadCounter
       055E  wCounter16_1_ReadCompareValue
       055E  _Counter16_1_wReadCompareValue
       055E  _Counter16_1_WriteCompareValue
       055E  _wCounter16_1_ReadCompareValue
       0598  _bE2Write
       0598  bE2Write
       072E  _E2Read
       072E  E2Read
       0794  _bFlashWriteBlock
       0794  bFlashWriteBlock
       0828  _FlashBlockLocal1
       0865  _FlashBlockLocal2
       08B5  _FlashBlockLocal3
       0985  __UserModules_end
       0985  _FlashReadBlock

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0985   15A8 =   5544. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0985  _SaveConfig
       0985  __text_start
       09D3  _ReadConfig
       0A0B  _GPIO_ISR
       0A20  _CYRF_Init
       0A68  _CYRF_WriteRegister
       0A81  _CYRF_WriteRegisterMulti
       0AE6  _CYRF_WriteRegisterMultiD
       0B41  _CYRF_WriteRegisterInc
       0B57  _CYRF_ReadRegister
       0B6F  _CYRF_ReadRegisterMulti
       0BF2  _CYRF_ReadRegisterInc
       0C08  _CYRF_GetMfgData
       0C2E  _CYRF_SetCRCSeed
       0C44  _Delay50uTimes
       0C44  Delay50uTimes
       0C4A  Delay50u_Table
       0C52  Delay50u
       0C52  _Delay50u
       0C66  _DSM_SetRangeCheck
       0C85  _SetTimer
       0CA2  _SetCH_CRC_SOP_DATA
       0E08  _DSM_Load_Config
       0E4D  _Bind_Xmit_cb
       0E6E  _Send_Xmit_cb
       0EB0  _DSM_StartReceive
       0EC2  _DSM_initialize_bind
       0FF2  _DSM_build_data_packet
       1060  _DSM_ClearRxError
       10B2  _Rcv_cb
       10D7  _DSM_StartBindRcv
       1109  _DSM_Cyclic
       1284  _DSM_NewSerialData
       133C  _calc_dsmx_channel
       15B7  _src_cb
       15BB  _find_dsm2_channel
       1688  _DSM_LoadTransmitMode
       16D0  _DSM_Init
       17B3  _GetMaxCh
       1820  _Ser_cb
       189A  _main
       1A94  _SerialSetCallback
       1AA0  _SerialRX_Init
       1AE3  _SerialDataReady
       1B01  _ser_Tic
       1B12  _RX8_1_Interrupt
       1BFF  _Timer_Wait_ms
       1C0E  _Timer_Enable
       1C31  _Timer_init
       1C4A  _Timer_GetTime
       1C75  _Timer_SetCallback
       1C81  _Sleep_Timer_Interrupt
       1CE6  _Counter16_1_Interrupt
       1D6D  PSoC_GPIO_ISR
       1ED5  __plcall
       1EE2  <created procedures>
       1F2D  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0000   000F =     15. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  __data_start
       0000  _cyrfmfg_id
       0006  dsm.c:tx_cmd
       0008  _ser_idx
       0009  _ser_status
       000A  _ser_to
       000B  _systic
       000D  _timer_cb
       000F  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
               virtual_registers   000F   000F =     15. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       000F  __rZ
       0010  __rY
       0011  __rX
       0012  __r11
       0013  __r10
       0014  __r9
       0015  __r8
       0016  __r7
       0017  __r6
       0018  __r5
       0019  __r4
       001A  __r3
       001B  __r2
       001C  __r1
       001D  __r0
       001E  __virtual_registers_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                    InterruptRAM   001E   0008 =      8. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       001E  SleepTimer_1_fTick
       001E  _SleepTimer_1_fTick
       001F  SleepTimer_1_bTimerValue
       001F  _SleepTimer_1_bTimerValue
       0020  SleepTimer_1_bCountDown
       0020  _SleepTimer_1_bCountDown
       0021  SleepTimer_1_TickCount
       0021  _SleepTimer_1_TickCount
       0023  _NO_SHADOW
       0023  _Port_0_Data_SHADE
       0023  Port_0_Data_SHADE
       0023  NO_SHADOW
       0024  _Port_1_Data_SHADE
       0024  Port_1_Data_SHADE
       0025  Port_3_Data_SHADE
       0025  _Port_3_Data_SHADE
       0026  __InterruptRAM_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0026   0071 =    113. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0026  _mem
       0026  __bss_start
       0046  _CY_irq
       0047  _cb_tick
       0048  _Last_Phase_error
       004A  _tx_cfg
       004B  _bind_channal
       004C  _State
       004D  _data_col
       004E  _sop_col
       004F  _Ch_A
       0050  _RX_Response
       0051  _chidx
       0052  _channels
       0069  _dsm_Max_pos
       006A  _DSM_Perriode
       006C  _DSM_ready_for_Serial
       006D  _DSM_Is_Forced
       006E  _DSM_Is_11Bit
       006F  _DSM_Is_11ms
       0070  _DSM_Is_DSMX
       0071  _Bind_cnt
       0072  _num_channels
       0074  _ModellNr
       0075  _ADD_CH_Map
       0076  _New_Ser_Data
       0077  _HighRes
       0078  _Ch_Data
       0085  _serial_cb
       0087  _ser_databuf
       0095  _cnt_last
       0097  __ramareas_end
       0097  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             TOP   0000   00F5 =    245. bytes (abs,ovr,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0068  __Start
       00F2  _bGetPowerSetting
       00F2  bGetPowerSetting

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                   E2PROM_1_AREA   1FC0   2000 =     64. bytes (abs,ovr,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       1FC0  _E2PROM_1_RESERVED
       1FC0  E2PROM_1_RESERVED

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                      SSCParmBlk   0000   0100 =    256. bytes (abs,ovr,ram)

Files Linked      [ module(s) ]

./obj/boot.o	[ boot.asm ]
obj/config.o	[ config.c ]
obj/cyrf6936.o	[ cyrf6936.c ]
obj/delay.o	[ delay.asm ]
obj/dsm.o	[ dsm.c ]
obj/main.o	[ main.c ]
obj/serial.o	[ serial.c ]
obj/timer.o	[ timer.c ]
<library>	[ spim_1.asm, sleeptimer_1int.asm, sleeptimer_1.asm, rx8_1int.asm, rx8_1.asm, rst.asm, psocgpioint.asm ]
[ psocconfigtbl.asm, psocconfig.asm, led_1.asm, e2prom_1.asm, cs_pin.asm, counter16_1int.asm ]
[ counter16_1.asm, E2PROMLIB.asm, flashblock.asm, rZ.asm, rY.asm, rX.asm, r11.asm, r10.asm, r9.asm ]
[ r8.asm, r7.asm, r6.asm, r5.asm, r4.asm, r3.asm, r2.asm, r1.asm, r0.asm, divmod_16x16_16_util.asm, divmodu_32x32_32_util.asm ]
[ divmodu_8x8_8_util.asm, i_div16_block_util.asm, i_div32_block_util.asm, i_div8_block_util.asm ]
[ mul16_util.asm, mul32_util.asm, plcall.s ]

Area Definitions

-blit:0x170.0x1fff
-bdata:0.0xff
-bSSCParmBlk:0xf7.0xff
-rwh:SSCParmBlk:0xf7.0xff

