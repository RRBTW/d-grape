#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       512U
#define USBD_DEBUG_LEVEL            0U
#define USBD_LPM_ENABLED            0U
#define USBD_SELF_POWERED           1U

#define CDC_DATA_HS_MAX_PACKET_SIZE  512U
#define CDC_DATA_FS_MAX_PACKET_SIZE   64U
#define CDC_CMD_PACKET_SIZE            8U
#define CDC_DATA_FS_IN_PACKET_SIZE   CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE  CDC_DATA_FS_MAX_PACKET_SIZE

#define USBD_malloc   malloc
#define USBD_free     free
#define USBD_memset   memset
#define USBD_memcpy   memcpy
#define USBD_Delay    HAL_Delay

void *USBD_static_malloc(uint32_t size);
void  USBD_static_free(void *p);