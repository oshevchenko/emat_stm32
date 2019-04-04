/*
 * timer_pulse.c
 *
 *  Created on: Apr 4, 2019
 *      Author: oleksandr
 */

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "event_queue.h"
#include "timer_pulse.h"

//Should be called from HW timer interrupt
static tm_pulse_sm_s timer;
static TIM_HandleTypeDef *m_htim;
static IRQn_Type m_irq;

void TIMER_PULSE_Start(uint32_t period)
{
	timer.state = P_RUNNING_BLOCK_AMP;
	m_htim->Instance->ARR = 15;
	m_htim->Instance->CNT = 0;
	HAL_TIM_Base_Start_IT(m_htim);
}

void TIMER_PULSE_Tick(void)
{
	if (P_RUNNING_BLOCK_AMP == timer.state) {
		m_htim->Instance->ARR = 285;
		(*timer.cb.p_amp_on_callback)();
		timer.state = P_RUNNING_PULSE;
	} else if (P_RUNNING_PULSE == timer.state) {
		(*timer.cb.p_pulse_off_callback)();
		timer.state = P_EXPIRED;
	}
}
void TIMER_PULSE_RegisterCallbackExpired(tm_pulse_cb_s *p_cb)
{
	timer.cb.p_amp_on_callback = p_cb->p_amp_on_callback;
	timer.cb.p_pulse_off_callback = p_cb->p_pulse_off_callback;
}

void TIMER_PULSE_Init(TIM_HandleTypeDef *htim, IRQn_Type irq)
{
	m_irq = irq;
	m_htim = htim;
}


void TIMER_PULSE_Task(void)
{
	tm_pulse_sm_e state;

	HAL_NVIC_DisableIRQ(m_irq);
	state = timer.state;
	if (P_EXPIRED == state) timer.state = P_IDLE;
	HAL_NVIC_EnableIRQ(m_irq);
	if (P_EXPIRED == state)
	{
		HAL_TIM_Base_Stop_IT(m_htim);
		EQ_PutEvent(PULSE_TIMER_EXPIRED);
	}
}
