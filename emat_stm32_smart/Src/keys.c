/*
 * keys.c
 *
 *  Created on: Apr 5, 2019
 *      Author: oleksandr
 */


#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "event_queue.h"
#include "keys.h"

static keys_sm_s key[KEYS_TOTAL];
static IRQn_Type m_irq;

void KEYS_Tick(void)
{
	static int c_key = 0;
	keys_sm_s *p_key;
	char pressed = 0;

	p_key = &key[c_key];
	c_key++;
	if (c_key == KEYS_TOTAL) c_key = 0;
	if (p_key->cb.p_check_key_callback) {
		pressed = (*p_key->cb.p_check_key_callback)();
	}
	p_key->filter <<= 1;
	if (pressed) p_key->filter |= 1;

}

void KEYS_Task(void)
{
	keys_sm_s *p_key;
	int i;
	int put_event = 0;

	for (i=0; i<KEYS_TOTAL; i++) {
		put_event = 0;
		p_key = &key[i];
		HAL_NVIC_DisableIRQ(m_irq);
		if ((K_RELEASED == p_key->state) && ((p_key->filter & p_key->cb.mask) == p_key->cb.mask)) {
			p_key->state = K_PRESSED;
			put_event = 1;
		}
		if ((K_PRESSED == p_key->state) && (0 == (p_key->filter & p_key->cb.mask))) {
			p_key->state = K_RELEASED;
		}
		HAL_NVIC_EnableIRQ(m_irq);
		if (put_event) {
			EQ_PutEvent(p_key->cb.event);
		}
	}
}
void KEYS_Init(IRQn_Type irq)
{
	m_irq = irq;
	keys_sm_s *p_key;
	int i;
	for (i=0; i<KEYS_TOTAL; i++) {
		p_key = &key[i];
		p_key->state = K_RELEASED;
		p_key->filter = 0;
		p_key->cb.mask = 0;
		p_key->cb.p_check_key_callback = NULL;
		p_key->cb.event = NO_EVENT;
	}

}

void KEYS_RegisterCallback(int n_key, keys_cb_s *p_cb)
{
	keys_sm_s *p_key;

	if (n_key >= KEYS_TOTAL) goto exit;
	p_key = &key[n_key];
	p_key->cb.mask = p_cb->mask;
	p_key->cb.p_check_key_callback = p_cb->p_check_key_callback;
	p_key->cb.event = p_cb->event;
exit:
	return;
}
