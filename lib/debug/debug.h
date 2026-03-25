#pragma once

/* ============================================================
 *  debug.h — Отладочный вывод состояния робота по USB CDC
 *
 *  Включается через #define DEBUG_MODE в robot_config.h.
 *  В этом режиме task_microros НЕ запускается — USB занят
 *  отладкой. task_debug печатает человекочитаемый отчёт
 *  каждые TASK_DEBUG_PERIOD_MS миллисекунд.
 *
 *  Как смотреть вывод:
 *    Windows : любой Serial Monitor (PlatformIO, Arduino IDE)
 *    Linux   : screen /dev/ttyACM0 115200
 *    Python  : python -m serial.tools.miniterm COMx 115200
 * ============================================================ */

#include "robot_config.h"

#ifdef DEBUG_MODE

#include <stdint.h>
#include "imu.h"
#include "kalman.h"

/* ── Полный снимок состояния системы ────────────────────────
 *  Заполняется в freertos_app.c и передаётся в debug_print().
 *  Все поля копируются под защитой мьютексов/critical section
 *  — debug модуль сам по себе никакого shared state не трогает.
 * ────────────────────────────────────────────────────────── */
typedef struct {
    uint32_t tick_ms;          /* HAL_GetTick() в момент снимка    */

    /* IMU (сырые физические единицы после конвертации) */
    float ax, ay, az;          /* акселерометр [м/с²]              */
    float gx, gy, gz;          /* гироскоп     [рад/с]             */
    float temp_c;              /* температура  [°C]                */
    uint8_t imu_valid;         /* 1 = данные свежие                */

    /* Энкодеры */
    float enc_left_mps;        /* скорость левой гусеницы  [м/с]  */
    float enc_right_mps;       /* скорость правой гусеницы [м/с]  */
    float enc_left_dist_m;     /* пройденное расстояние    [м]     */
    float enc_right_dist_m;

    /* Фильтр Калмана — выходные значения */
    float kf_vel_mps;          /* слитая линейная скорость [м/с]  */
    float kf_omega_rads;       /* слитая угловая скорость  [рад/с]*/
    float kf_accel_bias;       /* оценка bias акселя       [м/с²] */
    float kf_gyro_bias;        /* оценка bias гироскопа    [рад/с]*/

    /* Ковариация KF (мера уверенности) */
    float kf_p00_vel;          /* P[0][0] фильтра скорости         */
    float kf_p00_yaw;          /* P[0][0] фильтра угловой скорости */

    /* PID */
    float pid_setpoint_left;   /* уставка  левого мотора   [м/с]  */
    float pid_setpoint_right;
    float pid_out_left;        /* выход PID [-1..+1]               */
    float pid_out_right;
    float pid_integral_left;   /* накопленный интеграл             */
    float pid_integral_right;

    /* Команда от ROS / оператора */
    float cmd_left_mps;
    float cmd_right_mps;
    uint32_t last_cmd_ms;      /* когда пришла последняя команда   */

    /* USB CDC состояние */
    uint8_t  usb_configured;   /* 1 = USBD_STATE_CONFIGURED        */
    uint32_t usb_tx_errors;    /* сколько раз CDC_Transmit вернул BUSY/ERROR */
    uint32_t usb_rx_total;     /* всего принято байт               */
} DebugSnapshot_t;

/* ── API ────────────────────────────────────────────────────*/

/**
 * @brief  Инициализация модуля отладки.
 *         Вызвать один раз из freertos_app_init() перед стартом задач.
 */
void debug_init(void);

/**
 * @brief  Напечатать полный отчёт в USB CDC.
 *         Вызывается из task_debug каждые TASK_DEBUG_PERIOD_MS мс.
 * @param  s  указатель на заполненный снимок состояния
 */
void debug_print(const DebugSnapshot_t *s);

/**
 * @brief  Быстрый вывод произвольной строки (для разовых сообщений).
 *         Безопасно вызывать из любой задачи.
 * @param  msg  строка с завершающим \0
 */
void debug_log(const char *msg);

/**
 * @brief  Баннер при старте: версия прошивки, конфиг железа,
 *         параметры PID, тактирование, UID процессора.
 *         Вызвать один раз из task_debug до основного цикла.
 */
void debug_print_startup(void);

/* Счётчики для заполнения USB полей снимка состояния */
extern uint32_t          s_tx_errors_debug;
extern volatile uint32_t rx_total_debug;

#endif /* DEBUG_MODE */