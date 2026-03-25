#pragma once
#include "usbd_cdc.h"

#define CDC_RX_BUF_SIZE  512U
#define CDC_TX_BUF_SIZE  512U

extern USBD_CDC_ItfTypeDef USBD_CDC_fops;
extern uint8_t UserRxBufferFS[CDC_RX_BUF_SIZE];
extern uint8_t UserTxBufferFS[CDC_TX_BUF_SIZE];

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);