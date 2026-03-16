#pragma once

/* ============================================================
 *  console.h — Интерактивная консоль управления по USB CDC
 *
 *  Доступна только в DEBUG_MODE.
 *  Запускается как отдельная FreeRTOS задача (task_console),
 *  которая читает байты из rx_ring и парсит команды.
 *
 *  Два режима ввода:
 *
 *  1. Одиночные клавиши (без Enter) — мгновенное действие:
 *       w  вперёд       +CONSOLE_STEP_MPS к обеим гусеницам
 *       s  назад        -CONSOLE_STEP_MPS
 *       a  поворот влево  левая -step, правая +step
 *       d  поворот вправо левая +step, правая -step
 *     (пробел)  стоп
 *
 *  2. Текстовые команды + Enter:
 *       left <v> right <v>     — прямая уставка скоростей [м/с]
 *       stop                   — аварийный стоп (сброс PID)
 *       reset                  — обнулить энкодеры
 *       pid kp <v>             — изменить Kp
 *       pid ki <v>             — изменить Ki
 *       pid kd <v>             — изменить Kd
 *       help                   — список команд
 *
 *  Эхо набираемых символов включено — видно что печатаешь.
 * ============================================================ */

#include "robot_config.h"

#ifdef DEBUG_MODE

#include <stdint.h>

/* Шаг скорости для wasd-клавиш [м/с] */
#define CONSOLE_STEP_MPS        0.1f

/* Максимальная скорость через консоль [м/с] */
#define CONSOLE_MAX_MPS         1.5f

/* Размер буфера строки (команда + Enter) */
#define CONSOLE_LINE_BUF        64U

/* Стек и приоритет задачи */
#define TASK_CONSOLE_STACK      512U
#define TASK_CONSOLE_PRIORITY   osPriorityNormal

/* ── API ────────────────────────────────────────────────────*/

/**
 * @brief  Инициализация консоли.
 *         Вызвать из freertos_app_init() после debug_init().
 *         Принимает указатели на моторы и энкодеры чтобы не
 *         тянуть extern на внутренние static переменные.
 */
void console_init(void);

/**
 * @brief  Точка входа FreeRTOS задачи.
 *         Передать в osThreadNew как функцию задачи.
 */
void task_console(void *arg);

/**
 * @brief  Установить целевую скорость гусениц из консоли.
 *         Потокобезопасно (critical section внутри).
 */
void console_set_cmd(float left_mps, float right_mps);

#endif /* DEBUG_MODE */