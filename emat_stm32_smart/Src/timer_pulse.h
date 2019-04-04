/**
  ******************************************************************************
  * File Name          : timer_pulse.h
  * Description        : Microsecond scale timer.
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifndef __TIMER_PULSE_H__
#define __TIMER_PULSE_H__
typedef enum TIMER_PULSE_SM_STATE {
	P_IDLE = -1,
	P_RUNNING_BLOCK_AMP,
	P_RUNNING_PULSE,
	P_EXPIRED
} tm_pulse_sm_e;

typedef struct TIMER_PULSE_CB_STRUCT {
	void (*p_amp_on_callback)(void);
	void (*p_pulse_off_callback)(void);
} tm_pulse_cb_s;

typedef struct TIMER_PULSE_SM_STRUCT {
	tm_pulse_sm_e state;
	tm_pulse_cb_s cb;
//	void (*p_amp_on_callback)(void);
//	void (*p_pulse_off_callback)(void);
} tm_pulse_sm_s;

void TIMER_PULSE_Tick(void);
void TIMER_PULSE_Task(void);
void TIMER_PULSE_RegisterCallbackExpired(tm_pulse_cb_s *p_cb);
void TIMER_PULSE_Init(TIM_HandleTypeDef *htim, IRQn_Type irq);
void TIMER_PULSE_Start(uint32_t period);
#endif //#define __TIMER_PULSE_H__
/******END OF FILE****/

