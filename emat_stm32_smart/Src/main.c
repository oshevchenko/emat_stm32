
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "src/microrl.h"
#include "event_queue.h"
#include "timer.h"
#include "timer_pulse.h"
#include "keys.h"
#include "stm32_microrl_misc.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
//static uint32_t timer_counter;
static uint32_t pulse_width=10;
static uint32_t last_width=102;
typedef enum e_hv_state_TAG{
	HV_STATE_OFF,
	HV_STATE_ON
	} e_hv_state;
static e_hv_state hv_state = HV_STATE_OFF;
typedef enum e_pulse_state_TAG{
	PULSE_STATE_OFF,
	PULSE_STATE_DELAY_AFTER_HV_OFF,
	PULSE_STATE_ON_AFTER_HV_OFF,
	PULSE_STATE_ON
	} e_pulse_state;
static e_pulse_state pulse_state = PULSE_STATE_OFF;

typedef enum e_ext_int_state_TAG{
	EXT_INT_STATE_IDLE,
	EXT_INT_STATE_TRIGGERED,
	EXT_INT_STATE_DELAY_BEFORE_ON,
	EXT_INT_STATE_DELAY_BEFORE_OFF,
	} e_ext_int_state;
static e_ext_int_state ext_int_state = EXT_INT_STATE_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define HV_ON() { \
		HAL_GPIO_WritePin(GPIO_HV_ON_GPIO_Port, GPIO_HV_ON_Pin, GPIO_PIN_RESET); \
		HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_RESET); \
	}
#define HV_OFF() { \
		HAL_GPIO_WritePin(GPIO_HV_ON_GPIO_Port, GPIO_HV_ON_Pin, GPIO_PIN_SET); \
		HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_SET); \
    }

#define PULSE_ON() { \
		HAL_GPIO_WritePin(GPIO_PULSE_GPIO_Port, GPIO_PULSE_Pin, GPIO_PIN_RESET); \
		HAL_GPIO_WritePin(GPIO_PULSE_B_GPIO_Port, GPIO_PULSE_B_Pin, GPIO_PIN_RESET); \
	}
#define PULSE_OFF() { \
		HAL_GPIO_WritePin(GPIO_PULSE_GPIO_Port, GPIO_PULSE_Pin, GPIO_PIN_SET); \
		HAL_GPIO_WritePin(GPIO_PULSE_B_GPIO_Port, GPIO_PULSE_B_Pin, GPIO_PIN_SET); \
	}
#define PS_ON() { \
		HAL_GPIO_WritePin(GPIO_PS_EN_GPIO_Port, GPIO_PS_EN_Pin, GPIO_PIN_SET); \
	}
#define PS_OFF() { \
		HAL_GPIO_WritePin(GPIO_PS_EN_GPIO_Port, GPIO_PS_EN_Pin, GPIO_PIN_RESET); \
	}

#define AMP_ON() { \
		HAL_GPIO_WritePin(GPIO_AMP_ON_GPIO_Port, GPIO_AMP_ON_Pin, GPIO_PIN_SET); \
	}
#define AMP_OFF() { \
		HAL_GPIO_WritePin(GPIO_AMP_ON_GPIO_Port, GPIO_AMP_ON_Pin, GPIO_PIN_RESET); \
	}
//#define LED_GREEN_ON() HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_RESET)
//#define LED_GREEN_OFF() HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_SET)

#define DELAY_AFTER_HV_OFF 5

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void EventQueueIsFull(void)
{
	print("Error! EQ is Full!");
	for(;;);
}

void TimerPulseOnCallback(void)
{
	PULSE_ON();
}
void TimerPulseOffCallback(void)
{
	PULSE_OFF();
}
void TimerAmpOnCallback(void)
{
	AMP_ON();
}

char Key0Callback(void)
{
	char pressed = 0;
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_KEY1_GPIO_Port, GPIO_KEY1_Pin)){
		pressed = 1;
	}
	return pressed;
}

char KeyPA3Callback(void)
{
	char pressed = 0;
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_KEY_PA3_GPIO_Port, GPIO_KEY_PA3_Pin)){
		pressed = 1;
	}
	return pressed;
}

char KeyPA4Callback(void)
{
	char pressed = 0;
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_KEY_PA4_GPIO_Port, GPIO_KEY_PA4_Pin)){
		pressed = 1;
	}
	return pressed;
}

char KeyPA5Callback(void)
{
	char pressed = 0;
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_KEY_PA5_GPIO_Port, GPIO_KEY_PA5_Pin)){
		pressed = 1;
	}
	return pressed;
}

char KeyPA6Callback(void)
{
	char pressed = 0;
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIO_KEY_PA6_GPIO_Port, GPIO_KEY_PA6_Pin)){
		pressed = 1;
	}
	return pressed;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	eq_queue_element_s ev;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  KEYS_Init(TIM4_IRQn);
  keys_cb_s keys_cb;

  keys_cb.mask = 0x3FF;
  keys_cb.event = CMD_PULSE_ON;
  keys_cb.p_check_key_callback = &Key0Callback;
  KEYS_RegisterCallback(0, &keys_cb);

  keys_cb.mask = 0x3FF;
  keys_cb.event = CMD_HV_ON;
  keys_cb.p_check_key_callback = &KeyPA3Callback;
  KEYS_RegisterCallback(1, &keys_cb);

  keys_cb.mask = 0x3FF;
  keys_cb.event = CMD_PULSE_ON;
  keys_cb.p_check_key_callback = &KeyPA4Callback;
  KEYS_RegisterCallback(2, &keys_cb);

  keys_cb.mask = 0x3FF;
  keys_cb.event = CMD_HV_OFF;
  keys_cb.p_check_key_callback = &KeyPA5Callback;
  KEYS_RegisterCallback(3, &keys_cb);

  HAL_TIM_Base_Start_IT(&htim4);
//  HAL_TIM_Base_Start_IT(&htim3);
  init();
  TIMER_PULSE_Init(&htim3,TIM3_IRQn);
  tm_pulse_cb_s cb;
  cb.p_amp_on_callback = &TimerAmpOnCallback;
  cb.p_pulse_on_callback = &TimerPulseOnCallback;
  cb.p_pulse_off_callback = &TimerPulseOffCallback;
  cb.delay_to_pulse_on = 100;
  cb.delay_to_amp_on = 15;
  cb.delay_to_pulse_off = 285;
  TIMER_PULSE_RegisterCallbackExpired(&cb);
//  HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//	  HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_SET);
	  TERM_Task();
	  TIMER_Task();
	  TIMER_PULSE_Task();
	  KEYS_Task();
	  do {
		  EQ_GetEvent(&ev);
//		  ExtIntTask(&ev);
		  switch(ev.event){
		  case NO_EVENT:
			  break;
		  case CMD_WIDTH:
			  pulse_width=ev.param.uiParam;
//			  TIMER_StartAuto(1, ev.param.uiParam);
//			  last_width = ev.param.uiParam;
			  break;
		  case CMD_PULSE_OFF:
			  if (pulse_state == PULSE_STATE_ON) {
				  PULSE_OFF();
				  pulse_state = PULSE_STATE_OFF;
			  }
			  break;
		  case CMD_PULSE_ON:
			  if (hv_state == HV_STATE_ON) {
				  HV_OFF();
				  hv_state = HV_STATE_OFF;
				  PS_OFF();
				  AMP_OFF();
				  TIMER_PULSE_Start();
				  pulse_state = PULSE_STATE_DELAY_AFTER_HV_OFF;
			  } else {
//				  PULSE_ON();
				  AMP_OFF();
				  TIMER_PULSE_Start();

//				  TIMER_StartAuto(1, pulse_width);
				  pulse_state = PULSE_STATE_ON;
			  }
//			  HAL_GPIO_WritePin(GPIO_PULSE_GPIO_Port, GPIO_PULSE_Pin, GPIO_PIN_SET);  // SET means open IGBT
			  break;
		  case PULSE_TIMER_EXPIRED:
			  if (pulse_state == PULSE_STATE_ON) {
				  pulse_state = PULSE_STATE_OFF;
			  } else if (PULSE_STATE_DELAY_AFTER_HV_OFF == pulse_state) {
				  pulse_state = PULSE_STATE_OFF;
				  HV_ON();
				  hv_state = HV_STATE_ON;
				  PS_ON();
			  }
			  break;
		  case TIMER1_EXPIRED:
			  if (pulse_state == PULSE_STATE_ON) {
				  PULSE_OFF();
				  pulse_state = PULSE_STATE_OFF;
			  } else if (pulse_state == PULSE_STATE_DELAY_AFTER_HV_OFF) {
				  PS_OFF();
				  PULSE_ON();
				  TIMER_StartAuto(1, pulse_width);
				  pulse_state = PULSE_STATE_ON_AFTER_HV_OFF;
			  } else if (pulse_state == PULSE_STATE_ON_AFTER_HV_OFF) {
				  PULSE_OFF();
				  PS_ON();
				  pulse_state = PULSE_STATE_OFF;
				  HV_ON();
				  hv_state = HV_STATE_ON;
			  }

//			  HAL_GPIO_WritePin(GPIO_PULSE_GPIO_Port, GPIO_PULSE_Pin, GPIO_PIN_RESET);// RESET means close IGBT
			  break;
		  case CMD_HV_ON:
//			  HAL_GPIO_WritePin(HV_ON_OFF_GPIO_Port, HV_ON_OFF_Pin, GPIO_PIN_SET);
			  TIMER_Stop(1);
//			  LED_GREEN_ON();
			  PS_ON();
			  HV_ON();
			  hv_state = HV_STATE_ON;
			  PULSE_OFF();
			  pulse_state = PULSE_STATE_OFF;

//			  HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_RESET);
//			  HAL_GPIO_WritePin(GPIO_PULSE_GPIO_Port, GPIO_PULSE_Pin, GPIO_PIN_RESET);// RESET means close IGBT
//			  HAL_GPIO_WritePin(GPIO_HV_ON_GPIO_Port, GPIO_HV_ON_Pin, GPIO_PIN_RESET);
			  break;
		  case CMD_HV_OFF:
			  TIMER_Stop(1);
//			  LED_GREEN_OFF();
//			  HAL_GPIO_WritePin(GPIO_GREEN_LED_GPIO_Port, GPIO_GREEN_LED_Pin, GPIO_PIN_SET);
			  HV_OFF();
			  hv_state = HV_STATE_OFF;
//			  PULSE_ON();
			  pulse_state = PULSE_STATE_ON;
//			  HAL_GPIO_WritePin(GPIO_HV_ON_GPIO_Port, GPIO_HV_ON_Pin, GPIO_PIN_SET); // SET means OFF - close open drain
//			  HAL_GPIO_WritePin(GPIO_PULSE_GPIO_Port, GPIO_PULSE_Pin, GPIO_PIN_SET);
			  break;
		  default:
			  break;
		  }
	  } while (ev.event != NO_EVENT);

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 48;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 48000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_GREEN_LED_Pin|GPIO_HV_ON_Pin|GPIO_PULSE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_AMP_ON_Pin|GPIO_PS_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_PULSE_B_GPIO_Port, GPIO_PULSE_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : GPIO_GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GPIO_GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_HV_ON_Pin GPIO_PULSE_Pin */
  GPIO_InitStruct.Pin = GPIO_HV_ON_Pin|GPIO_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_KEY1_Pin GPIO_KEY_PA3_Pin GPIO_KEY_PA4_Pin GPIO_KEY_PA5_Pin 
                           GPIO_KEY_PA6_Pin */
  GPIO_InitStruct.Pin = GPIO_KEY1_Pin|GPIO_KEY_PA3_Pin|GPIO_KEY_PA4_Pin|GPIO_KEY_PA5_Pin 
                          |GPIO_KEY_PA6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_AMP_ON_Pin */
  GPIO_InitStruct.Pin = GPIO_AMP_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_AMP_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_PS_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_PS_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 PA9 PA10 
                           PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB4 PB5 PB6 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_PULSE_B_Pin */
  GPIO_InitStruct.Pin = GPIO_PULSE_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_PULSE_B_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void ReloadPulseTimer()
//{
//	htim3.Instance->ARR = 200;
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
