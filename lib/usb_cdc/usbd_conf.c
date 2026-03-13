#include "usbd_core.h"
#include "usbd_conf.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
{
    GPIO_InitTypeDef g = {0};
    if (pcdHandle->Instance != USB_OTG_FS) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin  = GPIO_PIN_9;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
{
    if (pcdHandle->Instance != USB_OTG_FS) return;
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *h)
    { USBD_LL_SetupStage(h->pData, (uint8_t *)h->Setup); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *h, uint8_t ep)
    { USBD_LL_DataOutStage(h->pData, ep, h->OUT_ep[ep].xfer_buff); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *h, uint8_t ep)
    { USBD_LL_DataInStage(h->pData, ep, h->IN_ep[ep].xfer_buff); }
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *h)
    { USBD_LL_SOF(h->pData); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *h)
    { USBD_LL_SetSpeed(h->pData, USBD_SPEED_FULL); USBD_LL_Reset(h->pData); }
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *h)
    { USBD_LL_Suspend(h->pData); }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *h)
    { USBD_LL_Resume(h->pData); }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *h)
    { USBD_LL_DevConnected(h->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *h)
    { USBD_LL_DevDisconnected(h->pData); }
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *h, uint8_t ep)
    { USBD_LL_IsoOUTIncomplete(h->pData, ep); }
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *h, uint8_t ep)
    { USBD_LL_IsoINIncomplete(h->pData, ep); }



USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_OTG_FS.pData    = pdev;
    pdev->pData              = &hpcd_USB_OTG_FS;
    hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints       = 4;
    hpcd_USB_OTG_FS.Init.speed               = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable          = DISABLE;
    hpcd_USB_OTG_FS.Init.phy_itface          = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable          = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable    = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable          = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1   = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) return USBD_FAIL;
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *p)
    { HAL_PCD_DeInit(p->pData); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *p)
    { HAL_PCD_Start(p->pData); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *p)
    { HAL_PCD_Stop(p->pData); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *p, uint8_t ep, uint8_t type, uint16_t mps)
    { HAL_PCD_EP_Open(p->pData, ep, mps, type); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *p, uint8_t ep)
    { HAL_PCD_EP_Close(p->pData, ep); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *p, uint8_t ep)
    { HAL_PCD_EP_Flush(p->pData, ep); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *p, uint8_t ep)
    { HAL_PCD_EP_SetStall(p->pData, ep); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *p, uint8_t ep)
    { HAL_PCD_EP_ClrStall(p->pData, ep); return USBD_OK; }
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *p, uint8_t ep)
{
    PCD_HandleTypeDef *h = p->pData;
    return (ep & 0x80) ? h->IN_ep[ep & 0x7F].is_stall
                       : h->OUT_ep[ep & 0x7F].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *p, uint8_t addr)
    { HAL_PCD_SetAddress(p->pData, addr); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *p, uint8_t ep, uint8_t *buf, uint32_t sz)
    { HAL_PCD_EP_Transmit(p->pData, ep, buf, sz); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *p, uint8_t ep, uint8_t *buf, uint32_t sz)
    { HAL_PCD_EP_Receive(p->pData, ep, buf, sz); return USBD_OK; }
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *p, uint8_t ep)
    { return HAL_PCD_EP_GetRxCount(p->pData, ep); }
void USBD_LL_Delay(uint32_t d) { HAL_Delay(d); }

void *USBD_static_malloc(uint32_t size)
    { static uint32_t mem[512/4]; (void)size; return mem; }
void USBD_static_free(void *p) { (void)p; }