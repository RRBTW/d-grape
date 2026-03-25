#pragma once

/* ============================================================
 *  motors.h — BTS7960B dual-PWM motor driver
 *
 *  Каждый мотор управляется двумя каналами ШИМ:
 *    htim_fwd / ch_fwd — RPWM (вперёд)
 *    htim_rev / ch_rev — LPWM (назад)
 *
 *  Основной API:
 *    motor_velocity_update(&m, target_mps, dt)
 *
 *  Без энкодеров — открытый контур (duty = target / MAX_MPS).
 *  С энкодерами — добавить PID на enc->speed_mps.
 *
 *  ⚠️ При подключении энкодеров: перенести левый LPWM
 *     с PA7 (TIM3_CH2) на PB10 (TIM2_CH3), чтобы освободить
 *     TIM3 для энкодера левого колеса.
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "encoders.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim_fwd;   /* таймер RPWM (вперёд) */
    uint32_t           ch_fwd;
    TIM_HandleTypeDef *htim_rev;   /* таймер LPWM (назад)  */
    uint32_t           ch_rev;
    volatile uint32_t *ccr_fwd;
    volatile uint32_t *ccr_rev;
    volatile uint32_t *arr;        /* ARR общий (оба таймера на одной частоте) */
    PID_t              pid;
    Encoder_t         *enc;        /* NULL — открытый контур */
} Motor_t;

void motor_init(Motor_t *m, Encoder_t *enc, float dt);
void motor_velocity_update(Motor_t *m, float target_mps, float dt);
void motor_stop(Motor_t *m);
