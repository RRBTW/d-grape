#pragma once
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
void MX_USB_DEVICE_Init(void);