#pragma once

/* ============================================================
 *  pid.h — PID регулятор с anti-windup и ограничением выхода
 * ============================================================ */

#include <stdint.h>

typedef struct {
    /* ── Коэффициенты ───────────────────────────────────────*/
    float kp;
    float ki;
    float kd;

    /* ── Ограничения ────────────────────────────────────────*/
    float output_max;       /* максимальный выход  */
    float output_min;       /* минимальный выход   */
    float integral_max;     /* anti-windup clamp   */

    /* ── Внутреннее состояние ───────────────────────────────*/
    float integral;
    float prev_error;
    float prev_measured;  /* для D-term на измерении */
    float dt;               /* шаг времени [с]     */

    /* ── Вход/выход ─────────────────────────────────────────*/
    float setpoint;         /* целевое значение    */
    float measured;         /* измеренное значение */
    float output;           /* выход регулятора    */

    uint8_t enabled;
} PID_t;

/* ── API ────────────────────────────────────────────────────*/

/** @brief Инициализация PID (обнуление состояния) */
void pid_init(PID_t *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_max, float dt);

/** @brief Один шаг регулятора. Результат в pid->output */
void pid_update(PID_t *pid);

/** @brief Сброс интегратора и состояния (при остановке) */
void pid_reset(PID_t *pid);

/** @brief Включить/выключить регулятор */
static inline void pid_enable(PID_t *pid)  { pid->enabled = 1; }
static inline void pid_disable(PID_t *pid) { pid->enabled = 0; pid_reset(pid); }