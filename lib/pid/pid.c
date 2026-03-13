#include "pid.h"

/* ─────────────────────────────────────────────────────────── */
void pid_init(PID_t *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_max, float dt)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->output_min   = output_min;
    pid->output_max   = output_max;
    pid->integral_max = integral_max;
    pid->dt           = dt;
    pid_reset(pid);
}

/* ─────────────────────────────────────────────────────────── */
void pid_update(PID_t *pid)
{
    if (!pid->enabled) {
        pid->output = 0.0f;
        return;
    }

    float error = pid->setpoint - pid->measured;

    /* P */
    float p_term = pid->kp * error;

    /* I с anti-windup (clamp интегратора) */
    pid->integral += error * pid->dt;
    if      (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* D на измерении (нет derivative kick при скачке setpoint) */
    float d_term = -pid->kd * (pid->measured - pid->prev_measured) / pid->dt;
    pid->prev_measured = pid->measured;

    /* Суммирование и ограничение выхода */
    float out = p_term + i_term + d_term;
    if      (out >  pid->output_max) out =  pid->output_max;
    else if (out <  pid->output_min) out =  pid->output_min;

    pid->output = out;
}

/* ─────────────────────────────────────────────────────────── */
void pid_reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error    = 0.0f;
    pid->prev_measured = 0.0f;
    pid->output     = 0.0f;
    pid->setpoint   = 0.0f;
    pid->measured   = 0.0f;
    pid->enabled    = 0;
}