#pragma once

/* ============================================================
 *  robot_config.h — Центральный конфигурационный файл D-Grape
 
 * ============================================================ */

#include "stm32f4xx_hal.h"

/* ── Физические параметры робота ────────────────────────────
 *  Используются в KF и encoders
 * ────────────────────────────────────────────────────────── */
#define ROBOT_WHEEL_RADIUS          0.0877f   /* [м] радиус звёздочки     */
#define ROBOT_TRACK_WIDTH           0.526f    /* [м] полная колея (2*LEN) */

/* ── Энкодеры ───────────────────────────────────────────────*/
/* TIM_ENCODERMODE_TI12 считает оба фронта обоих каналов (×4).
 * Физических импульсов на оборот = 1024, отсчётов таймера = 4096. */
#define ENCODER_PPR                 4096U
#define ENCODER_TIMER_PERIOD        0xFFFFU
#define ENCODER_LEFT_TIM            TIM3
#define ENCODER_RIGHT_TIM           TIM4
#define ENCODER_SKIS_TIM            TIM5

/* ── Моторы BTS7960B (dual PWM: RPWM=вперёд, LPWM=назад) ───
 *
 *  Левый:   PA15 → TIM2_CH1 (RPWM),  PA7  → TIM3_CH2 (LPWM)
 *  Правый:  PB3  → TIM2_CH2 (RPWM),  PB11 → TIM2_CH4 (LPWM)
 *  Лыжи:    PA8  → TIM1_CH1 (одиночный ШИМ, без реверса)
 *
 *  ⚠️ При подключении энкодеров: перенести левый LPWM
 *     с PA7 (TIM3_CH2) на PB10 (TIM2_CH3), чтобы TIM3 был
 *     свободен для энкодера левого колеса (PA6=A, PA7=B).
 * ────────────────────────────────────────────────────────── */
#define MOTOR_LEFT_CH_FWD       TIM_CHANNEL_1   /* PA15, TIM2 */
#define MOTOR_LEFT_CCR_FWD      (TIM2->CCR1)
#define MOTOR_LEFT_CH_REV       TIM_CHANNEL_2   /* PA7,  TIM3 */
#define MOTOR_LEFT_CCR_REV      (TIM3->CCR2)

#define MOTOR_RIGHT_CH_FWD      TIM_CHANNEL_2   /* PB3,  TIM2 */
#define MOTOR_RIGHT_CCR_FWD     (TIM2->CCR2)
#define MOTOR_RIGHT_CH_REV      TIM_CHANNEL_4   /* PB11, TIM2 */
#define MOTOR_RIGHT_CCR_REV     (TIM2->CCR4)

#define MOTOR_SKIS_CHANNEL      TIM_CHANNEL_1
#define MOTOR_SKIS_CCR          (TIM1->CCR1)
#define MOTOR_SKIS_DIR_PORT     GPIOD
#define MOTOR_SKIS_DIR_PIN      GPIO_PIN_2

/* ── PID (встроен в Motor_t) ────────────────────────────────*/
#define PID_KP                      0.3f
#define PID_KI                      0.0f
#define PID_KD                      0.0f
#define PID_OUTPUT_MAX              1.0f
#define PID_OUTPUT_MIN             -1.0f
#define PID_INTEGRAL_MAX            5.0f

/* ── IMU: MPU-6050 на I2C1 ──────────────────────────────────
 *  SCL → PB8 (AF4)
 *  SDA → PB9 (AF4)
 *  INT → PC0 (DRDY, опционально)
 *  AD0 = GND → I2C addr = 0x68
 * ────────────────────────────────────────────────────────── */
#define IMU_I2C_ADDR                (0x68U << 1)
#define IMU_I2C_TIMEOUT_MS          1U
#define IMU_ACCEL_FS_G              2             /* ±2g  → LSB/g = 16384  */
#define IMU_GYRO_FS_DPS             250           /* ±250°/s → LSB = 131   */
#define IMU_DLPF_CFG                3             /* LPF ~42 Гц            */
/* SMPLRT_DIV = 8000 / IMU_SAMPLE_RATE_HZ - 1 */
#define IMU_SMPLRT_DIV              1U            /* 1000/(1+1)=500 Гц при DLPF=3 */

/* ── Фильтр Калмана ─────────────────────────────────────────
 *
 *  KF1: линейная скорость — fusion (accel IMU + encoder avg)
 *    State x = [v, b_ax]  v[м/с], b_ax[м/с²] смещение акселя
 *
 *  KF2: угловая скорость — fusion (gyro gz + encoder diff)
 *    State x = [omega, b_gz]  omega[рад/с], b_gz[рад/с] смещение гироскопа
 *
 *  Увеличить R → больше доверия модели (IMU)
 *  Уменьшить R → больше доверия измерению (энкодеру)
 * ────────────────────────────────────────────────────────── */
#define KF_VEL_Q_V                  0.01f    /* шум процесса: скорость   */
#define KF_VEL_Q_BA                 0.001f   /* шум процесса: bias accel */
#define KF_VEL_R_ENC                0.05f    /* шум измерения: энкодер v */

#define KF_YAW_Q_W                  0.01f    /* шум процесса: omega      */
#define KF_YAW_Q_BG                 0.001f   /* шум процесса: bias gyro  */
#define KF_YAW_R_GYRO               0.1f     /* шум измерения: gyro gz   */
#define KF_YAW_R_ENC_W              0.05f    /* шум измерения: enc omega  */

/* ── FreeRTOS задачи ────────────────────────────────────────*/
#define TASK_IMU_STACK              768U
#define TASK_IMU_PRIORITY           osPriorityAboveNormal
#define TASK_IMU_PERIOD_MS          2U        /* 500 Гц */

#define TASK_ROBOT_STACK            1024U
#define TASK_ROBOT_PRIORITY         osPriorityNormal
#define TASK_ROBOT_PERIOD_MS        10U       /* 100 Гц */

#define TASK_MICROROS_STACK         4096U
#define TASK_MICROROS_PRIORITY      osPriorityNormal
#define TASK_MICROROS_PERIOD_MS     20U       /* 50 Гц  */

#define TASK_WATCHDOG_STACK         384U
#define TASK_WATCHDOG_PRIORITY      osPriorityHigh

/* ── Встроенные LED (STM32F407VG Discovery) ─────────────────
 *  PD12 — зелёный  LD4  IMU статус
 *  PD13 — оранжевый LD3  моторы активны
 *  PD14 — красный  LD5  heartbeat task_robot
 *  PD15 — синий    LD6  ROS агент / консоль
 *  PE1  — красный  LD7  авария / ошибка
 * ────────────────────────────────────────────────────────── */
#define LED_IMU_PORT        GPIOD
#define LED_IMU_PIN         GPIO_PIN_12   /* зелёный  — IMU OK  */

#define LED_MOTORS_PORT     GPIOD
#define LED_MOTORS_PIN      GPIO_PIN_13   /* оранжевый — моторы */

#define LED_HEARTBEAT_PORT  GPIOD
#define LED_HEARTBEAT_PIN   GPIO_PIN_14   /* красный  — 100 Гц  */

#define LED_COMM_PORT       GPIOD
#define LED_COMM_PIN        GPIO_PIN_15   /* синий    — связь   */

#define LED_ERROR_PORT      GPIOE
#define LED_ERROR_PIN       GPIO_PIN_1    /* красный  — авария  */
#define CMD_TIMEOUT_MS              5000U

/* ── micro-ROS ──────────────────────────────────────────────*/
#define MICROROS_AGENT_TIMEOUT_MS   500U
#define MICROROS_AGENT_ATTEMPTS     5U

/* ── Расчётные ──────────────────────────────────────────────*/
#define PID_DT_S   (TASK_ROBOT_PERIOD_MS / 1000.0f)
#define IMU_DT_S   (TASK_IMU_PERIOD_MS   / 1000.0f)

/* ── Отладочный режим ───────────────────────────────────────
 *
 *  Раскомментируй строку ниже чтобы включить debug-режим:
 *    - task_microros НЕ запускается (micro-ROS молчит)
 *    - вместо него стартует task_debug
 *    - task_debug каждые DEBUG_PERIOD_MS печатает в USB CDC
 *      человекочитаемый отчёт о состоянии всей системы
 *
 *  Смотреть вывод: любой Serial Monitor, 115200 бод,
 *  или: python -m serial.tools.miniterm COMx 115200
 * ────────────────────────────────────────────────────────── */
/* #define DEBUG_MODE*/

#ifdef DEBUG_MODE
#define TASK_DEBUG_STACK        1024U
#define TASK_DEBUG_PRIORITY     osPriorityNormal
#define TASK_DEBUG_PERIOD_MS    100U   /* 10 Гц — не перегружаем USB */
#endif