#include "uxr/client/transport.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define RX_RING_SIZE  512U
static uint8_t  rx_ring[RX_RING_SIZE];
static uint32_t rx_head = 0U;
static uint32_t rx_tail = 0U;

void microros_usb_recv_cb(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        rx_ring[rx_head % RX_RING_SIZE] = buf[i];
        rx_head++;
    }
}

bool usb_cdc_transport_open(struct uxrCustomTransport *t)
    { (void)t; rx_head = rx_tail = 0U; return true; }

bool usb_cdc_transport_close(struct uxrCustomTransport *t)
    { (void)t; return true; }

size_t usb_cdc_transport_write(struct uxrCustomTransport *t,
                               const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)t;
    uint32_t deadline = HAL_GetTick() + 20U;
    while (HAL_GetTick() < deadline) {
        if (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len) == USBD_OK)
            return len;
        osDelay(1);
    }
    *err = 1U;
    return 0U;
}

size_t usb_cdc_transport_read(struct uxrCustomTransport *t,
                              uint8_t *buf, size_t len,
                              int timeout_ms, uint8_t *err)
{
    (void)t;
    uint32_t deadline = HAL_GetTick() + (uint32_t)timeout_ms;
    size_t n = 0U;
    while (n < len && HAL_GetTick() < deadline) {
        if (rx_head != rx_tail) {
            buf[n++] = rx_ring[rx_tail % RX_RING_SIZE];
            rx_tail++;
        }
    }
    if (n == 0U) *err = 1U;
    return n;
}