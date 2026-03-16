#pragma once

/* ============================================================
 *  freertos_app.h — Публичный API FreeRTOS приложения D-Grape
 *
 *  Подключается из main.c и из lib/debug/ (console.c).
 *  Экспортирует глобальные объекты которые не static —
 *  консоль и debug-модуль обращаются к ним через extern.
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include "motors.h"
#include "encoders.h"

/* ── Точка входа ────────────────────────────────────────────
 *  Вызывается из main.c перед osKernelStart().
 *  Инициализирует аппаратуру, создаёт мьютексы и задачи.
 * ────────────────────────────────────────────────────────── */
void freertos_app_init(void);

/* ── Глобальные объекты (не static) ────────────────────────
 *  Объявлены без static в freertos_app.c чтобы консоль и
 *  debug-модуль могли обращаться к ним через extern.
 *
 *  Все обращения к volatile-переменным из разных задач
 *  защищены critical section (__disable_irq / __enable_irq).
 * ────────────────────────────────────────────────────────── */

/* Моторы с встроенным PID */
extern Motor_t   g_motor_left;
extern Motor_t   g_motor_right;
extern Motor_t   g_motor_skis;

/* Энкодеры */
extern Encoder_t g_enc_left;
extern Encoder_t g_enc_right;

/* Целевые скорости гусениц — пишет ROS callback или консоль,
 * читает task_robot. Защита: critical section. */
extern volatile float    g_cmd_left_mps;
extern volatile float    g_cmd_right_mps;
extern volatile uint32_t g_last_cmd_ms;