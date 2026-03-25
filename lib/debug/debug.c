/* ============================================================
 *  debug.c — Отладочный вывод состояния робота по USB CDC
 * ============================================================ */

#include "debug.h"

#ifdef DEBUG_MODE

#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include "robot_config.h"
#include "console.h"      /* CONSOLE_STEP_MPS */
#include "FreeRTOS.h"     /* xPortGetFreeHeapSize */
#include "task.h"
#include "stm32f4xx_hal.h"
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

/* ── Счётчик ошибок передачи USB ────────────────────────────*/
uint32_t s_tx_errors_debug = 0U;  /* видна из freertos_app.c */

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

    uint8_t sent = 0U;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 20U) {
        if (CDC_Transmit_FS((uint8_t *)s_buf, len) == USBD_OK) {
            sent = 1U;
            break;
        }
        osDelay(1);
    }
    if (!sent) s_tx_errors_debug++;

    osMutexRelease(s_usb_mutex);
}

/* ─────────────────────────────────────────────────────────── */
void debug_log(const char *msg)
{
    if (!msg) return;

    osMutexAcquire(s_usb_mutex, osWaitForever);

    uint16_t len = (uint16_t)strnlen(msg, DBG_BUF_SIZE - 1U);
    memcpy(s_buf, msg, len);

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 20U) {
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
    APPEND("\r\n[ENCODERS]  not connected\r\n");

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

    /* ── USB CDC ────────────────────────────────────────────── */
    APPEND("\r\n[USB CDC]\r\n");
    APPEND("  state      : %s\r\n",
           s->usb_configured ? "OK (CONFIGURED)" : "NOT CONFIGURED  <-- порт не подключён");
    APPEND("  tx_errors  : %lu  %s\r\n",
           (unsigned long)s->usb_tx_errors,
           (s->usb_tx_errors > 0) ? "<-- Serial Monitor не открыт?" : "");
    APPEND("  rx_total   : %lu байт\r\n",
           (unsigned long)s->usb_rx_total);

    APPEND("----------------------------------------------------------\r\n");

#undef APPEND

    usb_send((uint16_t)n);
    s_frame++;
}

/* ─────────────────────────────────────────────────────────── */
void debug_print_startup(void)
{
    /* STM32F4 уникальный 96-битный ID — три слова по 32 бита */
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    /* Тактирование — считываем из RCC напрямую */
    uint32_t sysclk_mhz = HAL_RCC_GetSysClockFreq() / 1000000U;
    uint32_t hclk_mhz   = HAL_RCC_GetHCLKFreq()     / 1000000U;
    uint32_t pclk1_mhz  = HAL_RCC_GetPCLK1Freq()    / 1000000U;
    uint32_t pclk2_mhz  = HAL_RCC_GetPCLK2Freq()    / 1000000U;

    /* Flash latency */
    uint32_t flash_lat  = (FLASH->ACR & FLASH_ACR_LATENCY);

    /* FreeRTOS heap */
    uint32_t heap_free  = (uint32_t)xPortGetFreeHeapSize();
    uint32_t heap_min   = (uint32_t)xPortGetMinimumEverFreeHeapSize();

    int n   = 0;
    int rem = (int)DBG_BUF_SIZE;

#define A(...) \
    do { int _w = snprintf(s_buf + n, (size_t)rem, __VA_ARGS__); \
         if (_w > 0) { n += _w; rem -= _w; } } while(0)

    A("\r\n");
    A("╔══════════════════════════════════════════════════╗\r\n");
    A("║          D-Grape  firmware  DEBUG MODE           ║\r\n");
    A("╚══════════════════════════════════════════════════╝\r\n");
    A("  Build   : %s  %s\r\n", __DATE__, __TIME__);
    A("  Board   : STM32F407VG Discovery\r\n");
    A("  UID     : %08lX-%08lX-%08lX\r\n",
      (unsigned long)uid0, (unsigned long)uid1, (unsigned long)uid2);

    A("\r\n");
    A("── Тактирование ────────────────────────────────────\r\n");
    A("  SYSCLK  : %3lu MHz  (PLL from HSE 8 MHz)\r\n", (unsigned long)sysclk_mhz);
    A("  HCLK    : %3lu MHz  AHB prescaler = /1\r\n",   (unsigned long)hclk_mhz);
    A("  PCLK1   : %3lu MHz  APB1 (TIM2-7 x2 = %lu MHz)\r\n",
      (unsigned long)pclk1_mhz, (unsigned long)pclk1_mhz * 2U);
    A("  PCLK2   : %3lu MHz  APB2 (TIM1,8 x2 = %lu MHz)\r\n",
      (unsigned long)pclk2_mhz, (unsigned long)pclk2_mhz * 2U);
    A("  Flash   : %lu wait-states\r\n", (unsigned long)flash_lat);

    A("\r\n");
    A("── Геометрия робота ────────────────────────────────\r\n");
    A("  Колесо  : r = %.4f m  (диаметр %.1f мм)\r\n",
      (double)ROBOT_WHEEL_RADIUS, (double)(ROBOT_WHEEL_RADIUS * 2000.0f));
    A("  Колея   : W = %.3f m\r\n", (double)ROBOT_TRACK_WIDTH);

    A("\r\n");
    A("── Энкодеры ────────────────────────────────────────\r\n");
    A("  Статус  : не подключены (open-loop режим)\r\n");
    A("  При подключении: TIM3 PA6/PA7 (левый), TIM4 PB6/PB7 (правый)\r\n");

    A("\r\n");
    A("── PID (начальные значения) ─────────────────────────\r\n");
    A("  Kp = %.4f   Ki = %.4f   Kd = %.4f\r\n",
      (double)PID_KP, (double)PID_KI, (double)PID_KD);
    A("  dt = %.4f s  out = [%.2f .. %.2f]\r\n",
      (double)PID_DT_S, (double)PID_OUTPUT_MIN, (double)PID_OUTPUT_MAX);
    A("  integral_max = %.2f\r\n", (double)PID_INTEGRAL_MAX);

    A("\r\n");
    A("── Задачи FreeRTOS ──────────────────────────────────\r\n");
    A("  task_imu     %4d ms  (IMU 500 Гц)\r\n",   TASK_IMU_PERIOD_MS);
    A("  task_robot   %4d ms  (control 100 Гц)\r\n", TASK_ROBOT_PERIOD_MS);
    A("  task_debug   %4d ms  (10 Гц)\r\n",         TASK_DEBUG_PERIOD_MS);
    A("  task_console  — интерактивная\r\n");
    A("  task_watchdog — watchdog 200 мс\r\n");

    A("\r\n");
    A("── Heap FreeRTOS ───────────────────────────────────\r\n");
    A("  Свободно  : %lu байт\r\n",   (unsigned long)heap_free);
    A("  Min ever  : %lu байт\r\n",   (unsigned long)heap_min);

    A("\r\n");
    A("── Консольные команды ──────────────────────────────\r\n");
    A("  w/s/a/d       — движение/поворот (%.2f м/с шаг)\r\n",
      (double)CONSOLE_STEP_MPS);
    A("  [пробел]      — стоп\r\n");
    A("  left <v> right <v>  — уставка скоростей [м/с]\r\n");
    A("  stop          — аварийный стоп, сброс PID\r\n");
    A("  reset         — сброс энкодеров\r\n");
    A("  pid kp/ki/kd <v>    — изменить PID коэффициент\r\n");
    A("  help          — повторить список команд\r\n");

    A("\r\n");
    A("── USB CDC ─────────────────────────────────────────\r\n");
    A("  VID:PID = 0483:5740  (STM Virtual COM Port)\r\n");
    A("  Скорость терминала: любая (CDC игнорирует baud)\r\n");

    A("\r\n");
    A("────────────────────────────────────────────────────\r\n");
    A("  Готов. Жду команды...\r\n");
    A("────────────────────────────────────────────────────\r\n");
    A("\r\n");

#undef A

    usb_send((uint16_t)n);
}

#endif /* DEBUG_MODE */