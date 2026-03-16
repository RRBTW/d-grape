#include "uxr/client/transport.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <string.h>

#define RX_RING_SIZE  512U
uint8_t           rx_ring[RX_RING_SIZE];
volatile uint32_t rx_head = 0U;  /* пишется из USB ISR */
volatile uint32_t rx_tail = 0U;  /* читается из task_microros */

/* Вызывается из USB ISR — не использует RTOS примитивы.
 * Доступ к rx_head/rx_tail защищён отключением прерываний на стороне
 * читателя (transport_read). Здесь достаточно volatile + единственного
 * writer-а (ISR), чтобы гарантировать видимость записи. */
void microros_usb_recv_cb(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        /* Защита от переполнения: не перезаписываем непрочитанные данные */
        if ((rx_head - rx_tail) < RX_RING_SIZE) {
            rx_ring[rx_head % RX_RING_SIZE] = buf[i];
            rx_head++;
        }
        /* При переполнении байт молча отбрасывается — это лучше, чем
         * затирать старые данные и получать мусорный micro-ROS фрейм */
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
        /* Читаем rx_head с отключёнными прерываниями, чтобы исключить
         * race: ISR может изменить rx_head между load и compare.
         * Секция очень короткая (2 инструкции) — не влияет на латентность USB. */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint32_t head = rx_head;
        uint32_t tail = rx_tail;
        __set_PRIMASK(primask);

        if (head != tail) {
            buf[n++] = rx_ring[tail % RX_RING_SIZE];
            /* rx_tail пишется только здесь (один reader) — volatile достаточно */
            rx_tail = tail + 1U;
        } else {
            /* Нет данных — уступаем CPU вместо busy-wait */
            osDelay(1);
        }
    }

    if (n == 0U) *err = 1U;
    return n;
}