#pragma once

/* ============================================================
 *  motors.h — Velocity-controlled motor driver
 *
 *  Архитектура v2: PID встроен в Motor_t.
 *  Снаружи нужен один вызов:
 *
 *    motor_velocity_update(&m, target_mps, dt)
 *
 *  который сам:
 *    1. Читает encoder->speed_mps как feedback
 *    2. Прогоняет PID (setpoint = target_mps, measured = enc speed)
 *    3. Пишет PWM + DIR
 *
 *  Прямой доступ к duty cycle сохранён через motor_set_duty()
 *  для лыж (у них нет энкодера → открытый контур).
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "encoders.h"
#include <stdint.h>

typedef struct {
    /* ── Аппаратура PWM ─────────────────────────────────────*/
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    volatile uint32_t *ccr;
    volatile uint32_t *arr;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;

    /* ── Встроенный PID ─────────────────────────────────────
     *  Инициализируется через motor_init() из robot_config.h
     *  Можно перенастроить через pid_init() после motor_init()
     * ────────────────────────────────────────────────────── */
    PID_t pid;

    /* ── Энкодер (только для ведущих моторов) ───────────────
     *  Если enc = NULL — мотор работает в open-loop режиме
     * ────────────────────────────────────────────────────── */
    Encoder_t *enc;
} Motor_t;

/* ── API ────────────────────────────────────────────────────*/

/**
 * @brief Инициализация мотора с PID
 * @param m    указатель на Motor_t
 * @param enc  указатель на Encoder_t (NULL для open-loop мотора)
 * @param dt   шаг времени PID [с] = TASK_ROBOT_PERIOD_MS / 1000.0f
 *
 * Запускает PWM, инициализирует PID из robot_config.h:
 *   KP = PID_KP, KI = PID_KI, KD = PID_KD
 *   output_min/max = PID_OUTPUT_MIN / PID_OUTPUT_MAX
 */
void motor_init(Motor_t *m, Encoder_t *enc, float dt);

/**
 * @brief Velocity control — основной API для ведущих моторов
 * @param m           указатель на Motor_t
 * @param target_mps  целевая скорость [м/с] (знак = направление)
 * @param dt          актуальный шаг [с] (из vTaskDelayUntil)
 *
 * Поток:
 *   pid.setpoint = target_mps
 *   pid.measured = m->enc->speed_mps   (из последнего encoder_update())
 *   pid_update(&m->pid)
 *   motor_set_duty(m, m->pid.output)
 *
 * Если m->enc == NULL — вызывает motor_set_duty(0) и возвращается.
 */
void motor_velocity_update(Motor_t *m, float target_mps, float dt);

/**
 * @brief Прямая установка скважности [-1..+1] (open-loop)
 *        Используется для лыж и аварийной остановки.
 */
void motor_set_duty(Motor_t *m, float duty);

/**
 * @brief Аварийная остановка (нулевое PWM, сброс PID)
 */
void motor_stop(Motor_t *m);
