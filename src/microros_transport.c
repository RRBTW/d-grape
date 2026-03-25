/* ============================================================
 *  microros_transport.c — USB CDC transport для micro-ROS
 *
 *  В DEBUG_MODE компилируется только ring-буфер + cdc_rx_hook.
 *  micro-ROS заголовки и функции транспорта — только без DEBUG_MODE.
 * ============================================================ */

#ifndef DEBUG_MODE
#  include "uxr/client/transport.h"
#  include "cmsis_os2.h"
#endif

#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define RX_RING_SIZE  512U
uint8_t           rx_ring[RX_RING_SIZE];
volatile uint32_t rx_head = 0U;  /* пишется из USB ISR */
volatile uint32_t rx_tail = 0U;  /* читается из task_microros */
volatile uint32_t rx_total_debug = 0U; /* всего принято байт — для debug */

/* Вызывается из USB ISR через weak cdc_rx_hook в usbd_cdc_if.c.
 * Не использует RTOS примитивы — только volatile + единственный writer. */
void cdc_rx_hook(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        if ((rx_head - rx_tail) < RX_RING_SIZE) {
            rx_ring[rx_head % RX_RING_SIZE] = buf[i];
            rx_head++;
            rx_total_debug++;
        }
    }
}

#ifndef DEBUG_MODE

bool usb_cdc_transport_open(struct uxrCustomTransport *t)
    { (void)t; rx_head = rx_tail = 0U; return true; }

bool usb_cdc_transport_close(struct uxrCustomTransport *t)
    { (void)t; return true; }

size_t usb_cdc_transport_write(struct uxrCustomTransport *t,
                               const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)t;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 20U) {
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
    uint32_t start = HAL_GetTick();
    size_t n = 0U;

    while (n < len && (HAL_GetTick() - start) < (uint32_t)timeout_ms) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint32_t head = rx_head;
        uint32_t tail = rx_tail;
        __set_PRIMASK(primask);

        if (head != tail) {
            buf[n++] = rx_ring[tail % RX_RING_SIZE];
            rx_tail = tail + 1U;
        } else {
            osDelay(1);
        }
    }

    if (n == 0U) *err = 1U;
    return n;
}

#endif /* ifndef DEBUG_MODE */
