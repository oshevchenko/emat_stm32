/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "stm32_microrl_misc.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  4
#define APP_TX_DATA_SIZE  1024 // I think this can be any value (was 2048)
#define IRQ_STATE_DISABLED (0x00000001)
#define IRQ_STATE_ENABLED  (0x00000000)
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
  static __IO uint8_t dev_is_connected = 1; // indicates if we are connected

  static uint8_t UserTxBuffer[APP_TX_DATA_SIZE]; // data for USB IN endpoind is stored in this buffer
  static uint16_t UserTxBufPtrIn = 0; // increment this pointer modulo APP_TX_DATA_SIZE when new data is available
  static __IO uint16_t UserTxBufPtrOut = 0; // increment this pointer modulo APP_TX_DATA_SIZE when data is drained
  static uint16_t UserTxBufPtrOutShadow = 0; // shadow of above
  static uint8_t UserTxBufPtrWaitCount = 0; // used to implement a timeout waiting for low-level USB driver
  static uint8_t UserTxNeedEmptyPacket = 0; // used to flush the USB IN endpoint if the last packet was exactly the endpoint packet size

//  static int user_interrupt_char = -1;
//  static void *user_interrupt_data = NULL;
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	int i;
	for (i = 0; i < *Len; i++) stm32_microrl_insert_char((int)UserRxBufferFS[i]);
	 USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &UserRxBufferFS[0]);
	 USBD_CDC_ReceivePacket(&hUsbDeviceFS);

//  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
//  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  if (!dev_is_connected) {
      // CDC device is not connected to a host, so we are unable to send any data
      return 1;
  }

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
// This function is called to process outgoing data.  We hook directly into the
// SOF (start of frame) callback so that it is called exactly at the time it is
// needed (reducing latency), and often enough (increasing bandwidth).
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
    if (!dev_is_connected) {
        // CDC device is not connected to a host, so we are unable to send any data
        return;
    }

    if (UserTxBufPtrOut == UserTxBufPtrIn && !UserTxNeedEmptyPacket) {
        // No outstanding data to send
        return;
    }

    if (UserTxBufPtrOut != UserTxBufPtrOutShadow) {
        // We have sent data and are waiting for the low-level USB driver to
        // finish sending it over the USB in-endpoint.
        // SOF occurs every 1ms, so we have a 150 * 1ms = 150ms timeout
        if (UserTxBufPtrWaitCount < 150) {
                UserTxBufPtrWaitCount++;
                return;
        }

//        if (UserTxBufPtrWaitCount < 150) {
//            USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
//            if (USBx_INEP(CDC_IN_EP & 0x7f)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ) {
//                // USB in-endpoint is still reading the data
//                UserTxBufPtrWaitCount++;
//                return;
//            }
//        }
        UserTxBufPtrOut = UserTxBufPtrOutShadow;
    }

    if (UserTxBufPtrOutShadow != UserTxBufPtrIn || UserTxNeedEmptyPacket) {
        uint32_t buffptr;
        uint32_t buffsize;

        if (UserTxBufPtrOutShadow > UserTxBufPtrIn) { // rollback
            buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOutShadow;
        } else {
            buffsize = UserTxBufPtrIn - UserTxBufPtrOutShadow;
        }

        buffptr = UserTxBufPtrOutShadow;

        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBuffer[buffptr], buffsize);

        if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK) {
            UserTxBufPtrOutShadow += buffsize;
            if (UserTxBufPtrOutShadow == APP_TX_DATA_SIZE) {
                UserTxBufPtrOutShadow = 0;
            }
            UserTxBufPtrWaitCount = 0;

            // According to the USB specification, a packet size of 64 bytes (CDC_DATA_FS_MAX_PACKET_SIZE)
            // gets held at the USB host until the next packet is sent.  This is because a
            // packet of maximum size is considered to be part of a longer chunk of data, and
            // the host waits for all data to arrive (ie, waits for a packet < max packet size).
            // To flush a packet of exactly max packet size, we need to send a zero-size packet.
            // See eg http://www.cypress.com/?id=4&rID=92719
            UserTxNeedEmptyPacket = (buffsize > 0 && buffsize % CDC_DATA_FS_MAX_PACKET_SIZE == 0 && UserTxBufPtrOutShadow == UserTxBufPtrIn);
        }
    }
}

// Always write all of the data to the device tx buffer, even if the
// device is not connected, or if the buffer is full.  Has a small timeout
// to wait for the buffer to be drained, in the case the device is connected.
void USBD_CDC_TxAlways(const uint8_t *buf, uint32_t len) {
    for (int i = 0; i < len; i++) {
        // If the CDC device is not connected to the host then we don't have anyone to receive our data.
        // The device may become connected in the future, so we should at least try to fill the buffer
        // and hope that it doesn't overflow by the time the device connects.
        // If the device is not connected then we should go ahead and fill the buffer straight away,
        // ignoring overflow.  Otherwise, we should make sure that we have enough room in the buffer.
        if (dev_is_connected) {
            // If the buffer is full, wait until it gets drained, with a timeout of 500ms
            // (wraparound of tick is taken care of by 2's complement arithmetic).
//            uint32_t start = HAL_GetTick();
//            while (((UserTxBufPtrIn + 1) & (APP_TX_DATA_SIZE - 1)) == UserTxBufPtrOut && HAL_GetTick() - start <= 500) {
//                if (query_irq() == IRQ_STATE_DISABLED) {
//                    // IRQs disabled so buffer will never be drained; exit loop
//                    break;
//                }
//                __WFI(); // enter sleep mode, waiting for interrupt
//            }

            // Some unused code that makes sure the low-level USB buffer is drained.
            // Waiting for low-level is handled in HAL_PCD_SOFCallback.
            /*
            start = HAL_GetTick();
            PCD_HandleTypeDef *hpcd = hUSBDDevice.pData;
            if (hpcd->IN_ep[0x83 & 0x7f].is_in) {
                //volatile uint32_t *xfer_count = &hpcd->IN_ep[0x83 & 0x7f].xfer_count;
                //volatile uint32_t *xfer_len = &hpcd->IN_ep[0x83 & 0x7f].xfer_len;
                USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
                while (
                    // *xfer_count < *xfer_len // using this works
                    // (USBx_INEP(3)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ) // using this works
                    && HAL_GetTick() - start <= 2000) {
                    __WFI(); // enter sleep mode, waiting for interrupt
                }
            }
            */
        }

        UserTxBuffer[UserTxBufPtrIn] = buf[i];
        UserTxBufPtrIn = (UserTxBufPtrIn + 1) & (APP_TX_DATA_SIZE - 1);
    }
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
