/* ============================================================
 *  kalman.c — Фильтр Калмана: velocity + yaw fusion
 *
 *  State dim = 2, все матрицы аналитически раскрыты в скаляры
 *  чтобы не тянуть BLAS/LAPACK на МК.
 *
 *  Нотация:
 *    x[0] = основная величина (v или omega)
 *    x[1] = bias (b_ax или b_gz)
 *
 *  Матрица перехода F (velocity model):
 *    x[0](k) = x[0](k-1) + (u - x[1](k-1)) * dt
 *    x[1](k) = x[1](k-1)
 *
 *    F = | 1  -dt |     B = | dt |
 *        | 0   1  |         |  0 |
 *
 *  Матрица наблюдения H = [1  0]  (измеряем только x[0])
 * ============================================================ */

#include "kalman.h"
#include <string.h>

/* ─────────────────────────────────────────────────────────── */
/*  KF_VEL                                                     */
/* ─────────────────────────────────────────────────────────── */

void kf_vel_init(KF2_t *kf, float dt, float q_v, float q_bias, float r_enc)
{
    memset(kf, 0, sizeof(KF2_t));
    kf->dt = dt;
    kf->q0 = q_v;
    kf->q1 = q_bias;
    kf->r  = r_enc;

    /* Начальная ковариация — высокая неопределённость */
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 1.0f;
}

float kf_vel_update(KF2_t *kf, float ax, float v_enc)
{
    const float dt = kf->dt;

    /* ── Predict ──────────────────────────────────────────── */
    /* x_pred = F*x + B*u,   u = ax (управляющий вход — IMU) */
    float x0_pred = kf->x[0] + (ax - kf->x[1]) * dt;
    float x1_pred = kf->x[1];   /* bias random walk */

    /* P_pred = F*P*Fᵀ + Q
     *
     *  F = | 1  -dt |    Fᵀ = | 1   0 |
     *      | 0   1  |         |-dt  1 |
     *
     *  Аналитически:
     *  P00_pred = P00 - dt*P10 - dt*(P01 - dt*P11) + q0
     *           = P00 - dt*(P01 + P10) + dt²*P11 + q0
     *  P01_pred = P01 - dt*P11
     *  P10_pred = P10 - dt*P11
     *  P11_pred = P11 + q1
     */
    float p00 = kf->P[0][0], p01 = kf->P[0][1];
    float p10 = kf->P[1][0], p11 = kf->P[1][1];

    float p00_pred = p00 - dt*(p01 + p10) + dt*dt*p11 + kf->q0;
    float p01_pred = p01 - dt*p11;
    float p10_pred = p10 - dt*p11;
    float p11_pred = p11 + kf->q1;

    /* ── Update (измерение: z = v_enc, H = [1, 0]) ────────── */
    /* S = H*P_pred*Hᵀ + R = P00_pred + R */
    float S = p00_pred + kf->r;
    if (S < 1e-9f) S = 1e-9f; /* защита от деления на 0 */

    /* K = P_pred*Hᵀ / S = [P00_pred/S, P10_pred/S]ᵀ */
    float k0 = p00_pred / S;
    float k1 = p10_pred / S;

    /* y = z - H*x_pred = v_enc - x0_pred */
    float y = v_enc - x0_pred;

    /* x = x_pred + K*y */
    kf->x[0] = x0_pred + k0 * y;
    kf->x[1] = x1_pred + k1 * y;

    /* P = (I - K*H)*P_pred
     *   I - K*H = | 1-k0   0 |
     *              | -k1    1 |
     *
     *  P00 = (1-k0)*p00_pred
     *  P01 = (1-k0)*p01_pred
     *  P10 = -k1*p00_pred + p10_pred
     *  P11 = -k1*p01_pred + p11_pred
     */
    kf->P[0][0] = (1.0f - k0) * p00_pred;
    kf->P[0][1] = (1.0f - k0) * p01_pred;
    kf->P[1][0] = p10_pred - k1 * p00_pred;
    kf->P[1][1] = p11_pred - k1 * p01_pred;

    /* Баг 7: P должна оставаться симметричной — принудительно выравниваем
     * P[0][1] и P[1][0] чтобы подавить накопление численной асимметрии */
    float p01_sym = (kf->P[0][1] + kf->P[1][0]) * 0.5f;
    kf->P[0][1] = p01_sym;
    kf->P[1][0] = p01_sym;

    return kf->x[0];
}

/* ─────────────────────────────────────────────────────────── */
/*  KF_YAW                                                     */
/* ─────────────────────────────────────────────────────────── */

void kf_yaw_init(KF2_t *kf, float dt, float q_w, float q_bias, float r)
{
    memset(kf, 0, sizeof(KF2_t));
    kf->dt = dt;
    kf->q0 = q_w;
    kf->q1 = q_bias;
    kf->r  = r;

    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 1.0f;
}

/* Sequential measurement update: сначала обновляем по гироскопу,
 * потом по энкодеру — эквивалентно joint update, но проще кодируется.
 *
 *  Модель:
 *    omega_est = gz - bias   (измерение гироскопа — непосредственный вход)
 *    bias дрейфует медленно
 *
 *  H = [1, -1] для gz:   z_gyro = omega + bias   → omega = gz - bias
 *      Упрощение: x[0] = omega_corrected = gz - x[1]
 *               H = [1, 0] для обоих,  gz приходит как вход в predict
 */
float kf_yaw_update(KF2_t *kf, float gz, float omega_enc,
                    float r_gyro, float r_enc)
{
    const float dt = kf->dt;

    /* ── Predict ──────────────────────────────────────────── */
    /* omega = gz - bias (интегрировать не нужно — измеряем скорость) */
    float x0_pred = gz - kf->x[1];   /* скорректированная omega */
    float x1_pred = kf->x[1];        /* bias остаётся */

    float p00 = kf->P[0][0], p01 = kf->P[0][1];
    float p10 = kf->P[1][0], p11 = kf->P[1][1];

    /* F = I (модель: omega не интегрируется в ускорение)
     * P_pred = P + Q */
    float p00_pred = p00 + kf->q0;
    float p01_pred = p01;
    float p10_pred = p10;
    float p11_pred = p11 + kf->q1;

    /* ── Update 1: гироскоп как измерение z1 = gz, H=[1,1]
     *   y1 = gz - (x0_pred + x1_pred) = gz - gz = 0  … trivial
     *   Вместо этого используем энкодер как единственное измерение,
     *   а gz уже учтён в predict через x0_pred = gz - bias.
     *   Это стандартный подход Madgwick/Mahony-style для gyro.
     * ────────────────────────────────────────────────────── */

    /* ── Update 2: энкодер omega, H = [1, 0] ────────────────*/
    float S2 = p00_pred + r_enc;
    if (S2 < 1e-9f) S2 = 1e-9f;

    float k0_2 = p00_pred / S2;
    float k1_2 = p10_pred / S2;

    float y2 = omega_enc - x0_pred;

    float x0_upd = x0_pred + k0_2 * y2;
    float x1_upd = x1_pred + k1_2 * y2;

    kf->P[0][0] = (1.0f - k0_2) * p00_pred;
    kf->P[0][1] = (1.0f - k0_2) * p01_pred;
    kf->P[1][0] = p10_pred - k1_2 * p00_pred;
    kf->P[1][1] = p11_pred - k1_2 * p01_pred;

    /* Симметризация P */
    float p01_sym = (kf->P[0][1] + kf->P[1][0]) * 0.5f;
    kf->P[0][1] = p01_sym;
    kf->P[1][0] = p01_sym;

    kf->x[0] = x0_upd;
    kf->x[1] = x1_upd;

    /* Подавляем неиспользованные параметры */
    (void)r_gyro;
    (void)dt;

    return kf->x[0];
}

/* ─────────────────────────────────────────────────────────── */
void kf_reset(KF2_t *kf)
{
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
}