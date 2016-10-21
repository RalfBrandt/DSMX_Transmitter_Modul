#ifndef TIMER_H
#define TIMER_H




//variables
extern WORD systic;

typedef void (*tmr_callback_t)(void);

#define Timer_Stop() Counter16_1_Stop()
#define Timer_Start() Counter16_1_Start()
#define Timer_SetCompare(value) Counter16_1_WriteCompareValue(value) 
#define Timer_SetPeriod(value) Counter16_1_WritePeriod(value)

//prototypes
void Timer_init (void);
void Timer_Wait_ms(BYTE ms);
void Timer_Enable(BOOL val);
void Timer_SetCallback(tmr_callback_t cb_fn);
WORD Timer_GetTime(void);

#endif //TIMER_H