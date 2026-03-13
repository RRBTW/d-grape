#pragma once

/* ============================================================
 *  main.h — D-Grape / STM32F407VG Discovery
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

/* ── HAL хэндлы (определены в main.c) ───────────────────────*/
extern TIM_HandleTypeDef htim1;   /* PWM  лыжи              */
extern TIM_HandleTypeDef htim2;   /* PWM  гусеницы          */
extern TIM_HandleTypeDef htim3;   /* Encoder левый          */
extern TIM_HandleTypeDef htim4;   /* Encoder правый         */
extern TIM_HandleTypeDef htim5;   /* Encoder лыжи           */
extern TIM_HandleTypeDef htim6;   /* HAL timebase           */
extern I2C_HandleTypeDef hi2c1;   /* MPU-6050               */

/* ── Error handler ──────────────────────────────────────────*/
void Error_Handler(void);