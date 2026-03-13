#pragma once

/* ============================================================
 *  imu.h — Драйвер MPU-6050 (I2C1)
 *
 *  Схема подключения:
 *    PB8 → SCL (I2C1, AF4)
 *    PB9 → SDA (I2C1, AF4)
 *    PC0 → INT (DRDY, опционально — не используется в polling режиме)
 *    AD0 → GND → I2C addr = 0x68
 *
 *  Режим работы: polling через HAL_I2C_Mem_Read
 *  Частота опроса: 500 Гц (управляется task_imu в FreeRTOS)
 *
 *  Выход: сырые данные IMU_Raw_t, готовые к передаче в KF
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── Регистры MPU-6050 ──────────────────────────────────────*/
#define MPU_REG_SMPLRT_DIV      0x19U
#define MPU_REG_CONFIG          0x1AU
#define MPU_REG_GYRO_CONFIG     0x1BU
#define MPU_REG_ACCEL_CONFIG    0x1CU
#define MPU_REG_ACCEL_XOUT_H    0x3BU   /* начало 14-байт блока данных */
#define MPU_REG_PWR_MGMT_1      0x6BU
#define MPU_REG_WHO_AM_I        0x75U
#define MPU_WHO_AM_I_EXPECTED   0x68U

/* ── Масштабные коэффициенты (зависят от FS в robot_config.h) ──
 *  При ±2g:    16384.0 LSB/g  → делим raw, умножаем на 9.80665
 *  При ±250°/s: 131.0 LSB/°/s → делим raw, умножаем на π/180
 * ────────────────────────────────────────────────────────── */
#define MPU_ACCEL_SCALE     (9.80665f / 16384.0f)  /* → м/с² */
#define MPU_GYRO_SCALE      (0.01745329f / 131.0f) /* → рад/с */

/* ── Структура сырых данных ─────────────────────────────────*/
typedef struct {
    float ax, ay, az;      /* ускорение [м/с²]  */
    float gx, gy, gz;      /* угловая скорость [рад/с] */
    float temp_c;           /* температура [°C]  */
    uint32_t timestamp_ms;  /* osKernelGetTickCount() */
    uint8_t  valid;         /* 1 = данные прочитаны успешно */
} IMU_Raw_t;

/* ── API ────────────────────────────────────────────────────*/

/**
 * @brief Инициализация MPU-6050
 * @param hi2c  указатель на HAL I2C handle (hi2c1)
 * @return HAL_OK / HAL_ERROR
 *
 * Выполняет:
 *   1. WHO_AM_I проверку
 *   2. Wake up (PWR_MGMT_1 = 0)
 *   3. Выбор тактирования PLL с гироскопом X (стабильнее)
 *   4. SMPLRT_DIV для 500 Гц
 *   5. DLPF конфигурацию
 *   6. Gyro FS = ±250°/s, Accel FS = ±2g
 */
HAL_StatusTypeDef imu_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Чтение одного кадра данных (14 байт за один I2C burst)
 * @param hi2c  указатель на HAL I2C handle
 * @param out   указатель на структуру результата
 * @return HAL_OK / HAL_ERROR
 *
 * Читает блок регистров 0x3B..0x48:
 *   ACCEL_X/Y/Z (6) + TEMP (2) + GYRO_X/Y/Z (6) = 14 байт
 * Конвертирует в физические единицы.
 */
HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *hi2c, IMU_Raw_t *out);
