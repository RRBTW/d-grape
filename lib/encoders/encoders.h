#pragma once

/* ============================================================
 *  encoders.h — Квадратурные энкодеры через таймеры STM32
 * ============================================================ */

#include "stm32f4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;        /* таймер в encoder mode  */
    int32_t            count_prev;  /* предыдущее значение    */
    float              speed_mps;   /* текущая скорость [м/с] */
    float              distance_m;  /* пройденное расстояние  */
} Encoder_t;

/* ── API ────────────────────────────────────────────────────*/

/** @brief Запуск таймера в режиме encoder */
void encoder_init(Encoder_t *enc);

/**
 * @brief Обновление скорости — вызывать каждые dt секунд
 * @param enc  энкодер
 * @param dt   период обновления [с]
 */
void encoder_update(Encoder_t *enc, float dt);

/** @brief Сброс счётчика и накопленных данных */
void encoder_reset(Encoder_t *enc);
