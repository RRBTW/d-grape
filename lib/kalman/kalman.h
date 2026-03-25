#pragma once

/* ============================================================
 *  kalman.h — Два фильтра Калмана для fusion IMU + энкодер
 *
 *  KF1 — KF_Vel: линейная скорость
 *  ─────────────────────────────────────────────────────────
 *  State x = [v, b_ax]ᵀ
 *    v    — линейная скорость [м/с]
 *    b_ax — смещение акселерометра [м/с²] (медленный дрейф)
 *
 *  Модель процесса (dt = шаг задачи):
 *    v(k)    = v(k-1)    + (ax(k) - b_ax(k-1)) * dt
 *    b_ax(k) = b_ax(k-1)                    (random walk)
 *
 *  Измерение z = v_enc = (v_left + v_right) / 2
 *    H = [1, 0]
 *
 *  KF2 — KF_Yaw: угловая скорость рыскания
 *  ─────────────────────────────────────────────────────────
 *  State x = [omega, b_gz]ᵀ
 *    omega — угловая скорость [рад/с]
 *    b_gz  — смещение гироскопа [рад/с]
 *
 *  Модель процесса:
 *    omega(k) = omega(k-1) + (gz(k) - b_gz(k-1)) * dt  ← НЕ используется
 *    Упрощение: измеряем gz напрямую, корректируем bias
 *
 *  Два источника измерений:
 *    z1 = gz_imu    (гироскоп, высокочастотный)
 *    z2 = omega_enc = (v_right - v_left) / track_width  (энкодер, медленный)
 *    H1 = H2 = [1, 1]  и [1, 0]
 * ============================================================ */

#include <stdint.h>

/* ── Структура общего 2x2 скалярного KF ─────────────────────
 *  State dimension = 2 (достаточно для velocity bias estimation)
 *  Все матрицы хранятся в row-major [строка][столбец]
 * ────────────────────────────────────────────────────────── */
typedef struct {
    /* Состояние */
    float x[2];         /* x[0] = основная величина, x[1] = bias */

    /* Ковариация ошибки состояния */
    float P[2][2];

    /* Шумы процесса (диагональ Q) */
    float q0;           /* Q[0][0] — шум x[0]  */
    float q1;           /* Q[1][1] — шум x[1]  */

    /* Шум измерения */
    float r;            /* R (скаляр) — один источник измерения */

    float dt;           /* шаг времени [с] */
} KF2_t;

/* ── Выходная структура фильтрованных данных IMU ────────────*/
typedef struct {
    float velocity_mps;     /* KF1 выход: линейная скорость [м/с] */
    float omega_rads;       /* KF2 выход: угловая скорость [рад/с] */
    float accel_bias;       /* оценка смещения акселерометра */
    float gyro_bias;        /* оценка смещения гироскопа */
    float kf_p00_vel;       /* P[0][0] фильтра скорости (мера неопределённости) */
    float kf_p00_yaw;       /* P[0][0] фильтра угловой скорости */
    uint8_t valid;
} KF_Output_t;

/* ── API ────────────────────────────────────────────────────*/

/**
 * @brief Инициализация KF фильтра скорости
 * @param kf      указатель на структуру
 * @param dt      шаг интегрирования [с]
 * @param q_v     шум процесса скорости
 * @param q_bias  шум процесса смещения
 * @param r_enc   шум измерения энкодера
 */
void kf_vel_init(KF2_t *kf, float dt, float q_v, float q_bias, float r_enc);

/**
 * @brief Один шаг KF скорости
 * @param kf        указатель на структуру
 * @param ax        ускорение по X от IMU [м/с²]
 * @param v_enc     скорость по энкодерам (avg) [м/с]
 * @return          отфильтрованная скорость [м/с]
 *
 * Алгоритм:
 *   Predict: x_pred = F*x + B*u (u = ax - bias)
 *            P_pred = F*P*Fᵀ + Q
 *   Update:  K = P_pred*Hᵀ / (H*P_pred*Hᵀ + R)
 *            x = x_pred + K*(z - H*x_pred)
 *            P = (I - K*H)*P_pred
 */
float kf_vel_update(KF2_t *kf, float ax, float v_enc);

/**
 * @brief Инициализация KF фильтра угловой скорости
 * @param kf      указатель на структуру
 * @param dt      шаг интегрирования [с]
 * @param q_w     шум процесса omega
 * @param q_bias  шум процесса смещения гироскопа
 * @param r       шум измерения (используется R_enc, gyro как вход)
 */
void kf_yaw_init(KF2_t *kf, float dt, float q_w, float q_bias, float r);

/**
 * @brief Один шаг KF угловой скорости (sequential update из 2 источников)
 * @param kf        указатель на структуру
 * @param gz        угловая скорость гироскопа [рад/с]
 * @param omega_enc (v_right - v_left) / track_width [рад/с]
 * @param r_gyro    шум измерения гироскопа (из robot_config.h)
 * @param r_enc     шум измерения энкодерного omega
 * @return          отфильтрованная угловая скорость [рад/с]
 */
float kf_yaw_update(KF2_t *kf, float gz, float omega_enc,
                    float r_gyro, float r_enc);

/**
 * @brief Только шаг предсказания, без коррекции измерением.
 *        Использовать когда энкодер не подключён.
 */
float kf_vel_predict_only(KF2_t *kf, float ax);
float kf_yaw_predict_only(KF2_t *kf, float gz);

/**
 * @brief Сброс фильтра (при потере связи или остановке)
 */
void kf_reset(KF2_t *kf);
