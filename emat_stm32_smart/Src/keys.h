/*
 * keys.h
 *
 *  Created on: Apr 5, 2019
 *      Author: oleksandr
 */

#ifndef __KEYS_H__
#define __KEYS_H__
#include "event_queue.h"

#define KEYS_TOTAL 10

typedef enum KEYS_SM_STATE {
	K_IDLE = -1,
	K_PRESSED,
	K_RELEASED
} keys_sm_e;

typedef struct KEYS_CB_STRUCT {
	uint32_t mask;
	char (*p_check_key_callback)(void);
	eq_queue_event_e event;
} keys_cb_s;

typedef struct KEYS_SM_STRUCT {
	keys_sm_e state;
	keys_cb_s cb;
	uint32_t filter;
} keys_sm_s;

void KEYS_Tick(void);
void KEYS_Task(void);
void KEYS_RegisterCallback(int n_key, keys_cb_s *p_cb);
void KEYS_Init(IRQn_Type irq);

#endif /* __KEYS_H__ */
