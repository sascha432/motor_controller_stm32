/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <string.h>

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

/** Received data over USB are stored in this buffer      */
#define CDC_BINARY_BUFFER_SIZE (APP_RX_DATA_SIZE - 16)

static uint8_t UserBinaryBufferFS[CDC_BINARY_BUFFER_SIZE];
static volatile uint16_t UserBinaryBufferLengthFS;

enum
{
  CDC_BINARY_HEADER_SIZE = 12,
  CDC_BINARY_MAGIC = 0xDEADBEEF,
};

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

volatile uint32_t usbTerminalConnected;

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
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, NULL, 0);
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

    case CDC_SET_LINE_CODING:
    break;

    case CDC_GET_LINE_CODING:
    break;

    case CDC_SET_CONTROL_LINE_STATE:
    usbTerminalConnected = ((USBD_SetupReqTypedef *)pbuf)->wValue & 0x01;
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
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  uint16_t received = (uint16_t)*Len;

  if (received > 0U)
  {
    uint16_t binary_space = (uint16_t)(sizeof(UserBinaryBufferFS) - UserBinaryBufferLengthFS);
    if (received <= binary_space)
    {
      memcpy(&UserBinaryBufferFS[UserBinaryBufferLengthFS], Buf, received);
      UserBinaryBufferLengthFS = (uint16_t)(UserBinaryBufferLengthFS + received);
    }
    else if (received <= sizeof(UserBinaryBufferFS))
    {
      memcpy(UserBinaryBufferFS, Buf, received);
      UserBinaryBufferLengthFS = received;
    }
    else
    {
      UserBinaryBufferLengthFS = 0;
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
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
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint16_t CDC_ReadBinary_FS(uint8_t* Buf, uint16_t Len, uint16_t *Type, uint32_t *Crc)
{
  uint16_t available = UserBinaryBufferLengthFS;
  uint16_t magic_offset = 0;

  if (available == 0U)
  {
    return 0U;
  }

  while ((uint16_t)(magic_offset + sizeof(uint32_t)) <= available)
  {
    if (*(const uint32_t *)&UserBinaryBufferFS[magic_offset] == CDC_BINARY_MAGIC)
    {
      break;
    }
    ++magic_offset;
  }

  if ((uint16_t)(magic_offset + sizeof(uint32_t)) > available)
  {
    UserBinaryBufferLengthFS = 0U;
    return 0U;
  }

  if (magic_offset > 0U)
  {
    uint16_t remaining = (uint16_t)(available - magic_offset);
    memmove(UserBinaryBufferFS, &UserBinaryBufferFS[magic_offset], remaining);
    UserBinaryBufferLengthFS = remaining;
    available = remaining;
  }

  if (available < CDC_BINARY_HEADER_SIZE)
  {
    return 0U;
  }

  const uint16_t payload_size = *(const uint16_t *)&UserBinaryBufferFS[4];
  const uint16_t binary_type = *(const uint16_t *)&UserBinaryBufferFS[6];
  const uint32_t crc = *(const uint32_t *)&UserBinaryBufferFS[8];
  const uint16_t frame_size = (uint16_t)(CDC_BINARY_HEADER_SIZE + payload_size);

  if (payload_size > (sizeof(UserBinaryBufferFS) - CDC_BINARY_HEADER_SIZE))
  {
    UserBinaryBufferLengthFS = 0U;
    return 0U;
  }

  if (available < frame_size)
  {
    return 0U;
  }

  if (Len < payload_size)
  {
    return 0U;
  }

  memcpy(Buf, &UserBinaryBufferFS[CDC_BINARY_HEADER_SIZE], payload_size);

  uint16_t remaining = (uint16_t)(available - frame_size);
  if (remaining > 0U)
  {
    memmove(UserBinaryBufferFS, &UserBinaryBufferFS[frame_size], remaining);
  }
  UserBinaryBufferLengthFS = remaining;

  if (Type != NULL)
  {
    *Type = binary_type;
  }

  if (Crc != NULL)
  {
    *Crc = crc;
  }

  return payload_size;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
