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

/* SVC, PendSV, SysTick — переопределены FreeRTOS через FreeRTOSConfig.h */

/* ── TIM6 — HAL Timebase 1 кГц ─────────────────────────────*/
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/* ── USB OTG FS ─────────────────────────────────────────────
 *  Подключается после интеграции micro-ROS USB CDC транспорта.
 *  hpcd_USB_OTG_FS будет объявлен в usb_device.c из
 *  micro_ros_stm32cubemx_utils.
 * ────────────────────────────────────────────────────────── */

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
