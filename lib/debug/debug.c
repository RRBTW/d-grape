/* ============================================================
 *  debug.c — Отладочный вывод состояния робота по USB CDC
 * ============================================================ */

#include "debug.h"

#ifdef DEBUG_MODE

#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

/* ── Внутренний буфер ───────────────────────────────────────
 *  snprintf пишет сюда, потом одним вызовом CDC_Transmit_FS.
 *  Размер подобран с запасом под самый длинный отчёт (~600 байт)
 *  и не выходит за стек задачи (TASK_DEBUG_STACK = 512 слов = 2 КБ).
 * ────────────────────────────────────────────────────────── */
#define DBG_BUF_SIZE  768U
static char s_buf[DBG_BUF_SIZE];

/* ── Мьютекс для доступа к USB CDC ─────────────────────────
 *  CDC_Transmit_FS не реентерабельна — защищаем её,
 *  чтобы debug_log() из разных задач не конфликтовали.
 * ────────────────────────────────────────────────────────── */
static osMutexId_t s_usb_mutex;

/* ── Счётчик кадров ─────────────────────────────────────────*/
static uint32_t s_frame = 0U;

/* ─────────────────────────────────────────────────────────── */
void debug_init(void)
{
    s_usb_mutex = osMutexNew(NULL);
    s_frame     = 0U;
}

/* ─────────────────────────────────────────────────────────── */
/*  Вспомогательная функция: отправить s_buf по USB с защитой  */
/* ─────────────────────────────────────────────────────────── */
static void usb_send(uint16_t len)
{
    if (len == 0U) return;

    osMutexAcquire(s_usb_mutex, osWaitForever);

    /* CDC_Transmit_FS может вернуть USBD_BUSY если предыдущая
     * передача ещё не завершена — ждём до 20 мс, потом сдаёмся */
    uint32_t deadline = HAL_GetTick() + 20U;
    while (HAL_GetTick() < deadline) {
        if (CDC_Transmit_FS((uint8_t *)s_buf, len) == USBD_OK)
            break;
        osDelay(1);
    }

    osMutexRelease(s_usb_mutex);
}

/* ─────────────────────────────────────────────────────────── */
void debug_log(const char *msg)
{
    if (!msg) return;

    osMutexAcquire(s_usb_mutex, osWaitForever);

    uint16_t len = (uint16_t)strnlen(msg, DBG_BUF_SIZE - 1U);
    memcpy(s_buf, msg, len);

    uint32_t deadline = HAL_GetTick() + 20U;
    while (HAL_GetTick() < deadline) {
        if (CDC_Transmit_FS((uint8_t *)s_buf, len) == USBD_OK)
            break;
        osDelay(1);
    }

    osMutexRelease(s_usb_mutex);
}

/* ─────────────────────────────────────────────────────────── */
void debug_print(const DebugSnapshot_t *s)
{
    if (!s) return;

    int n = 0;
    int rem = (int)DBG_BUF_SIZE;

/* Макрос: добавить строку в буфер, сдвинуть указатель.
 * Прекращает запись если буфер кончился (не UB). */
#define APPEND(...) \
    do { \
        int _w = snprintf(s_buf + n, (size_t)rem, __VA_ARGS__); \
        if (_w > 0) { n += _w; rem -= _w; } \
    } while (0)

    /* ── Заголовок ─────────────────────────────────────────── */
    APPEND("\r\n");
    APPEND("========== D-Grape debug [frame %lu | t=%lu ms] ==========\r\n",
           (unsigned long)s_frame, (unsigned long)s->tick_ms);

    /* ── IMU ───────────────────────────────────────────────── */
    APPEND("\r\n[IMU]  %s\r\n", s->imu_valid ? "OK" : "ERROR / no data");
    APPEND("  accel  ax=%7.3f  ay=%7.3f  az=%7.3f  m/s2\r\n",
           (double)s->ax, (double)s->ay, (double)s->az);
    APPEND("  gyro   gx=%7.3f  gy=%7.3f  gz=%7.3f  rad/s\r\n",
           (double)s->gx, (double)s->gy, (double)s->gz);
    APPEND("  temp   %.1f C\r\n", (double)s->temp_c);

    /* ── Энкодеры ──────────────────────────────────────────── */
    APPEND("\r\n[ENCODERS]\r\n");
    APPEND("  left   speed=%7.3f m/s   dist=%8.3f m\r\n",
           (double)s->enc_left_mps,  (double)s->enc_left_dist_m);
    APPEND("  right  speed=%7.3f m/s   dist=%8.3f m\r\n",
           (double)s->enc_right_mps, (double)s->enc_right_dist_m);

    /* ── Фильтр Калмана ────────────────────────────────────── */
    APPEND("\r\n[KALMAN FILTER]\r\n");
    APPEND("  velocity   fused=%7.3f m/s    P00=%.5f\r\n",
           (double)s->kf_vel_mps,   (double)s->kf_p00_vel);
    APPEND("  omega      fused=%7.3f rad/s  P00=%.5f\r\n",
           (double)s->kf_omega_rads,(double)s->kf_p00_yaw);
    APPEND("  bias       accel=%7.4f m/s2   gyro=%7.4f rad/s\r\n",
           (double)s->kf_accel_bias,(double)s->kf_gyro_bias);

    /* ── PID ───────────────────────────────────────────────── */
    APPEND("\r\n[PID]\r\n");
    APPEND("  left   set=%6.3f  meas=%6.3f  out=%6.3f  int=%7.4f\r\n",
           (double)s->pid_setpoint_left,
           (double)s->enc_left_mps,
           (double)s->pid_out_left,
           (double)s->pid_integral_left);
    APPEND("  right  set=%6.3f  meas=%6.3f  out=%6.3f  int=%7.4f\r\n",
           (double)s->pid_setpoint_right,
           (double)s->enc_right_mps,
           (double)s->pid_out_right,
           (double)s->pid_integral_right);

    /* ── Команда ───────────────────────────────────────────── */
    APPEND("\r\n[COMMAND]\r\n");
    uint32_t age_ms = s->tick_ms - s->last_cmd_ms;
    APPEND("  left=%6.3f m/s   right=%6.3f m/s   age=%lu ms%s\r\n",
           (double)s->cmd_left_mps,
           (double)s->cmd_right_mps,
           (unsigned long)age_ms,
           (age_ms > 500U) ? "  [TIMEOUT]" : "");

    APPEND("----------------------------------------------------------\r\n");

#undef APPEND

    usb_send((uint16_t)n);
    s_frame++;
}

#endif /* DEBUG_MODE */