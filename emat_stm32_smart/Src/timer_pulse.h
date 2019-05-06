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
	P_RUN_TO_PS_OFF,
	P_RUN_TO_PULSE_ON,
	P_RUN_TO_AMP_ON,
	P_RUN_TO_PULSE_OFF,
	P_EXPIRED
} tm_pulse_sm_e;

typedef struct TIMER_PULSE_CB_STRUCT {
	uint32_t delay_to_ps_off;
	uint32_t delay_to_pulse_on;
	uint32_t delay_to_amp_on;
	uint32_t delay_to_pulse_off;
	void (*p_ps_off_callback)(void);
	void (*p_amp_on_callback)(void);
	void (*p_pulse_on_callback)(void);
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
void TIMER_PULSE_Start(void);
#endif //#define __TIMER_PULSE_H__
/******END OF FILE****/

