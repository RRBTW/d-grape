/* ============================================================
 *  motors.c — BTS7960B dual-PWM motor driver
 * ============================================================ */

#include "motors.h"
#include "robot_config.h"

#define MOTOR_MAX_MPS  1.5f   /* максимальная скорость [м/с] для нормировки */

/* Применить скважность [-1..+1] к парам FWD/REV каналов */
static void motor_apply_duty(Motor_t *m, float duty)
{
    if      (duty >  1.0f) duty =  1.0f;
    else if (duty < -1.0f) duty = -1.0f;

    uint32_t period = *m->arr;
    if (duty >= 0.0f) {
        *m->ccr_fwd = (uint32_t)(duty * (float)period);
        *m->ccr_rev = 0U;
    } else {
        *m->ccr_fwd = 0U;
        *m->ccr_rev = (uint32_t)(-duty * (float)period);
    }
}

void motor_init(Motor_t *m, Encoder_t *enc, float dt)
{
    m->enc = enc;

    pid_init(&m->pid,
             PID_KP, PID_KI, PID_KD,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX,
             PID_INTEGRAL_MAX,
             dt);
    pid_enable(&m->pid);

    /* Каналы запускаются в MX_TIM*_Init до motor_init.
     * Здесь только обнуляем выходы. */
    motor_apply_duty(m, 0.0f);
}

void motor_velocity_update(Motor_t *m, float target_mps, float dt)
{
    (void)dt;
    /* Открытый контур: скважность пропорциональна уставке.
     * Когда энкодеры подключены — заменить на PID:
     *   m->pid.setpoint = target_mps;
     *   m->pid.measured = m->enc->speed_mps;
     *   m->pid.dt = dt;
     *   pid_update(&m->pid);
     *   motor_apply_duty(m, m->pid.output);
     */
    float duty = target_mps / MOTOR_MAX_MPS;
    if      (duty >  1.0f) duty =  1.0f;
    else if (duty < -1.0f) duty = -1.0f;
    motor_apply_duty(m, duty);
}

void motor_stop(Motor_t *m)
{
    *m->ccr_fwd = 0U;
    *m->ccr_rev = 0U;
    pid_reset(&m->pid);
    pid_enable(&m->pid);
}
