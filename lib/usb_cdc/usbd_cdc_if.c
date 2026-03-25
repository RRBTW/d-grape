#include "usbd_cdc_if.h"

uint8_t UserRxBufferFS[CDC_RX_BUF_SIZE];
uint8_t UserTxBufferFS[CDC_TX_BUF_SIZE];

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *pLen);

USBD_CDC_ItfTypeDef USBD_CDC_fops = {
    CDC_Init_FS, CDC_DeInit_FS, CDC_Control_FS, CDC_Receive_FS
};

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void) { return USBD_OK; }

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)cmd; (void)pbuf; (void)length;
    return USBD_OK;
}

__attribute__((weak)) void cdc_rx_hook(uint8_t *buf, uint32_t len)
{ (void)buf; (void)len; }

static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *pLen)
{
    cdc_rx_hook(pbuf, *pLen);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len)
{
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0U) return USBD_BUSY;
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buf, len);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}