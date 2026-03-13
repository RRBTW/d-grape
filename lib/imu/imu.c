/* ============================================================
 *  imu.c — Драйвер MPU-6050
 * ============================================================ */

#include "imu.h"
#include "robot_config.h"
#include "cmsis_os2.h"
#include <string.h>

/* ── Локальный хелпер: запись одного байта в регистр ────────*/
static HAL_StatusTypeDef mpu_write(I2C_HandleTypeDef *hi2c,
                                   uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c,
                             IMU_I2C_ADDR,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1,
                             IMU_I2C_TIMEOUT_MS);
}

/* ── Инициализация ──────────────────────────────────────────*/
HAL_StatusTypeDef imu_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;
    HAL_StatusTypeDef st;

    /* 1. Проверка WHO_AM_I */
    st = HAL_I2C_Mem_Read(hi2c,
                          IMU_I2C_ADDR,
                          MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                          &who, 1,
                          IMU_I2C_TIMEOUT_MS);
    if (st != HAL_OK || who != MPU_WHO_AM_I_EXPECTED)
        return HAL_ERROR;

    /* 2. Сброс устройства */
    st = mpu_write(hi2c, MPU_REG_PWR_MGMT_1, 0x80U); /* DEVICE_RESET */
    if (st != HAL_OK) return st;
    osDelay(100); /* ждём завершения reset */

    /* 3. Выход из sleep, тактирование от PLL гироскопа X */
    st = mpu_write(hi2c, MPU_REG_PWR_MGMT_1, 0x01U); /* CLKSEL = 1 */
    if (st != HAL_OK) return st;
    osDelay(10);

    /* 4. Sample Rate Divider: SMPLRT_DIV = 15 → 8000/(15+1) = 500 Гц */
    st = mpu_write(hi2c, MPU_REG_SMPLRT_DIV, (uint8_t)IMU_SMPLRT_DIV);
    if (st != HAL_OK) return st;

    /* 5. DLPF: CONFIG = 3 → Accel 44 Гц, Gyro 42 Гц
     *    (при DLPF != 0 gyro rate = 1 кГц, sample rate = 1000/(1+SMPLRT_DIV))
     *    Для 500 Гц: SMPLRT_DIV = 1
     *    Без DLPF (CONFIG=0) gyro rate = 8 кГц, SMPLRT_DIV = 15 → 500 Гц */
    st = mpu_write(hi2c, MPU_REG_CONFIG, (uint8_t)IMU_DLPF_CFG);
    if (st != HAL_OK) return st;

    /* 6. Gyro Full Scale = ±250°/s (FS_SEL = 0) */
    st = mpu_write(hi2c, MPU_REG_GYRO_CONFIG, 0x00U);
    if (st != HAL_OK) return st;

    /* 7. Accel Full Scale = ±2g (AFS_SEL = 0) */
    st = mpu_write(hi2c, MPU_REG_ACCEL_CONFIG, 0x00U);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

/* ── Чтение одного кадра ────────────────────────────────────*/
HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *hi2c, IMU_Raw_t *out)
{
    uint8_t buf[14];

    /* Burst-чтение 14 байт начиная с ACCEL_XOUT_H (0x3B):
     *  [0..1]  ACCEL_X   [2..3]  ACCEL_Y   [4..5]  ACCEL_Z
     *  [6..7]  TEMP
     *  [8..9]  GYRO_X    [10..11] GYRO_Y   [12..13] GYRO_Z
     */
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c,
                                             IMU_I2C_ADDR,
                                             MPU_REG_ACCEL_XOUT_H,
                                             I2C_MEMADD_SIZE_8BIT,
                                             buf, sizeof(buf),
                                             IMU_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        out->valid = 0;
        return st;
    }

    /* Сборка 16-бит знаковых значений (big-endian) */
    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t raw_t  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* Конвертация в физические единицы */
    out->ax = (float)raw_ax * MPU_ACCEL_SCALE;
    out->ay = (float)raw_ay * MPU_ACCEL_SCALE;
    out->az = (float)raw_az * MPU_ACCEL_SCALE;

    out->gx = (float)raw_gx * MPU_GYRO_SCALE;
    out->gy = (float)raw_gy * MPU_GYRO_SCALE;
    out->gz = (float)raw_gz * MPU_GYRO_SCALE;

    /* Температура: T[°C] = raw / 340.0 + 36.53 */
    out->temp_c = (float)raw_t / 340.0f + 36.53f;

    out->timestamp_ms = osKernelGetTickCount();
    out->valid = 1;

    return HAL_OK;
}