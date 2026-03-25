/* ============================================================
 *  stm32f4xx_it.c — Обработчики прерываний
 * ============================================================ */

#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── Системные исключения ───────────────────────────────────*/
void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void MemManage_Handler(void)  { while (1) {} }
void BusFault_Handler(void)   { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }

/* SVC, PendSV, SysTick — определены в cmsis_os2.c и port.c (FreeRTOS) */

/* ── USB OTG FS ─────────────────────────────────────────────
 * ────────────────────────────────────────────────────────── */

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}