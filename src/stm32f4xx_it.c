/* ============================================================
 *  stm32f4xx_it.c — Обработчики прерываний
 * ============================================================ */

#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── Вспомогательный макрос: мигание LED в fault-обработчике ─
 *  PE1 (красный) — N быстрых вспышек, затем долгая пауза.
 *  Работает без FreeRTOS и без HAL_GetTick (чистый счётчик). */
#define FAULT_BLINK_FAST  1680000UL   /* ~10 мс при 168 МГц */
#define FAULT_BLINK_SLOW  16800000UL  /* ~100 мс */

static inline void fault_spin(uint8_t blinks)
{
    __disable_irq();
    /* Выключить все LED, кроме PE1 */
    HAL_GPIO_WritePin(GPIOD,
        GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    for (;;) {
        for (uint8_t i = 0; i < blinks; i++) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
            for (volatile uint32_t d = 0; d < FAULT_BLINK_FAST; d++) {}
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
            for (volatile uint32_t d = 0; d < FAULT_BLINK_FAST; d++) {}
        }
        /* Долгая пауза между группами */
        for (volatile uint32_t d = 0; d < FAULT_BLINK_SLOW * 3; d++) {}
    }
}

/* ── Системные исключения ───────────────────────────────────
 *  Каждый fault имеет уникальный код мигания PE1:
 *    HardFault   — 1 вспышка   (самый частый)
 *    MemManage   — 2 вспышки
 *    BusFault    — 3 вспышки
 *    UsageFault  — 4 вспышки
 *    NMI         — 5 вспышек
 * ────────────────────────────────────────────────────────── */
void NMI_Handler(void)        { fault_spin(5); }
void HardFault_Handler(void)  { fault_spin(1); }
void MemManage_Handler(void)  { fault_spin(2); }
void BusFault_Handler(void)   { fault_spin(3); }
void UsageFault_Handler(void) { fault_spin(4); }

/* ── FreeRTOS application hooks ─────────────────────────────*/

/* Tick hook: вызывается каждые 1 мс из FreeRTOS SysTick.
 * Обновляет uwTick чтобы HAL_GetTick() / HAL_Delay() работали
 * корректно после osKernelStart() — без этого I2C timeout
 * замерзает и task_imu блокирует task_robot. */
void vApplicationTickHook(void)
{
    HAL_IncTick();
}

/* Stack overflow hook: задача переполнила стек.
 * PE1 горит постоянно (без мигания) — отличие от fault. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __disable_irq();
    HAL_GPIO_WritePin(GPIOD,
        GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    while (1) {}
}

/* Malloc failed hook: FreeRTOS heap исчерпан.
 * PD15 (синий) + PE1 (красный) мигают вместе. */
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOD,
        GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
    for (;;) {
        HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_15,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1,   GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < FAULT_BLINK_FAST * 5; d++) {}
        HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_15,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_1,   GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < FAULT_BLINK_FAST * 5; d++) {}
    }
}

/* SVC, PendSV, SysTick — определены в cmsis_os2.c и port.c (FreeRTOS) */

/* ── USB OTG FS ─────────────────────────────────────────────
 * ────────────────────────────────────────────────────────── */

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}