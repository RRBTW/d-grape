#include "encoders.h"
#include "robot_config.h"

/* ─────────────────────────────────────────────────────────── */
void encoder_init(Encoder_t *enc)
{
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
    encoder_reset(enc);
}

/* ─────────────────────────────────────────────────────────── */
void encoder_update(Encoder_t *enc, float dt)
{
    int32_t count_now = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);

    /* Обработка переполнения 16-битного счётчика */
    int32_t delta = count_now - enc->count_prev;
    if      (delta >  (int32_t)(ENCODER_TIMER_PERIOD / 2)) delta -= (int32_t)ENCODER_TIMER_PERIOD + 1;
    else if (delta < -(int32_t)(ENCODER_TIMER_PERIOD / 2)) delta += (int32_t)ENCODER_TIMER_PERIOD + 1;

    enc->count_prev = count_now;

    /* Перевод импульсов → скорость [м/с]
     * v = (delta / PPR) * 2π * R / dt
     * Упрощённо: v = delta * (2π * R) / (PPR * dt)
     */
    float revolutions = (float)delta / (float)ENCODER_PPR;
    enc->speed_mps   = revolutions * (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS) / dt;
    enc->distance_m += revolutions * (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS);
}

/* ─────────────────────────────────────────────────────────── */
void encoder_reset(Encoder_t *enc)
{
    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->count_prev = 0;
    enc->speed_mps  = 0.0f;
    enc->distance_m = 0.0f;
}