/* ============================================================
 *  freertos_app.c — D-Grape firmware v2
 * ============================================================ */

#include "freertos_app.h"
#include "robot_config.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include <math.h>   /* isfinite() */

#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "kalman.h"

#ifdef DEBUG_MODE
#  include "debug.h"
#  include "console.h"
#else
/* rcl_publish() объявлен с warn_unused_result — (void) не помогает в GCC.
 * Присваивание во временную переменную гарантированно гасит варнинг. */
#  define RCL_PUB(pub, msg) do { rcl_ret_t _r = rcl_publish((pub),(msg),NULL); (void)_r; } while(0)
#  include <rcl/rcl.h>
#  include <rcl/error_handling.h>
#  include <rclc/rclc.h>
#  include <rclc/executor.h>
#  include <rmw_microros/rmw_microros.h>
#  include <geometry_msgs/msg/vector3.h>
#  include <sensor_msgs/msg/imu.h>
#  include <diagnostic_msgs/msg/diagnostic_array.h>
#endif

#include "usb_device.h"

/* HAL handles */
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5;
extern I2C_HandleTypeDef hi2c1;

/* ══════════════════════════════════════════════════════════ */
/*  ГЛОБАЛЬНОЕ СОСТОЯНИЕ                                      */
/* ══════════════════════════════════════════════════════════ */

volatile float    g_cmd_left_mps  = 0.0f;
volatile float    g_cmd_right_mps = 0.0f;
volatile uint32_t g_last_cmd_ms   = 0;

static IMU_Raw_t         g_imu_raw;
static osMutexId_t       g_imu_mutex;

static KF_Output_t       g_kf_out;
static osMutexId_t       g_kf_mutex;

static volatile float    g_enc_left_mps  = 0.0f;
static volatile float    g_enc_right_mps = 0.0f;

static osThreadId_t      g_watchdog_handle;

/* ══════════════════════════════════════════════════════════ */
/*  АППАРАТНЫЕ ОБЪЕКТЫ                                        */
/* ══════════════════════════════════════════════════════════ */

Encoder_t g_enc_left  = { .htim = &htim3 };
Encoder_t g_enc_right = { .htim = &htim4 };

Motor_t g_motor_left = {
    .htim     = &htim2,
    .channel  = MOTOR_LEFT_CHANNEL,
    .ccr      = &MOTOR_LEFT_CCR,
    .arr      = &TIM2->ARR,
    .dir_port = MOTOR_LEFT_DIR_PORT,
    .dir_pin  = MOTOR_LEFT_DIR_PIN,
};
Motor_t g_motor_right = {
    .htim     = &htim2,
    .channel  = MOTOR_RIGHT_CHANNEL,
    .ccr      = &MOTOR_RIGHT_CCR,
    .arr      = &TIM2->ARR,
    .dir_port = MOTOR_RIGHT_DIR_PORT,
    .dir_pin  = MOTOR_RIGHT_DIR_PIN,
};
Motor_t g_motor_skis = {
    .htim     = &htim1,
    .channel  = MOTOR_SKIS_CHANNEL,
    .ccr      = &MOTOR_SKIS_CCR,
    .arr      = &TIM1->ARR,
    .dir_port = MOTOR_SKIS_DIR_PORT,
    .dir_pin  = MOTOR_SKIS_DIR_PIN,
};

static KF2_t g_kf_vel;
static KF2_t g_kf_yaw;

/* ══════════════════════════════════════════════════════════ */
/*  micro-ROS  (не компилируется в DEBUG_MODE)                */
/* ══════════════════════════════════════════════════════════ */
#ifndef DEBUG_MODE

static rcl_node_t          g_node;
static rcl_allocator_t     g_allocator;
static rclc_support_t      g_support;
static rclc_executor_t     g_executor;

static rcl_publisher_t     g_pub_imu;
static rcl_publisher_t     g_pub_vel;
static rcl_publisher_t     g_pub_diag;
static rcl_subscription_t  g_sub_wheel_cmd;

static sensor_msgs__msg__Imu                 g_msg_imu;
static geometry_msgs__msg__Vector3           g_msg_vel;
static diagnostic_msgs__msg__DiagnosticArray g_msg_diag;
static geometry_msgs__msg__Vector3           g_msg_wheel_cmd;

/* ── Callback: wheel_cmd ─────────────────────────────────── */
static void wheel_cmd_callback(const void *msg_in)
{
    const geometry_msgs__msg__Vector3 *cmd =
        (const geometry_msgs__msg__Vector3 *)msg_in;

    float l = cmd->x, r = cmd->y;

    /* Баг 3: isfinite() ловит и NaN, и ±Inf — оба недопустимы как уставка */
    if (!isfinite(l)) l = 0.0f;
    if (!isfinite(r)) r = 0.0f;

    /* Clamp */
    const float MAX_MPS = 1.5f;
    if (l >  MAX_MPS) l =  MAX_MPS;
    if (l < -MAX_MPS) l = -MAX_MPS;
    if (r >  MAX_MPS) r =  MAX_MPS;
    if (r < -MAX_MPS) r = -MAX_MPS;

    /* Баг 9: float не атомарен — защищаем запись пары значений отключением прерываний.
     * Секция минимальная: 3 записи + восстановление PRIMASK. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_cmd_left_mps  = l;
    g_cmd_right_mps = r;
    g_last_cmd_ms   = osKernelGetTickCount();
    __set_PRIMASK(primask);

    /* Флаг 0x01 убран: watchdog его не ждёт, он был бессмысленным (баг 6).
     * Watchdog пингуется только из task_robot (флаг 0x02) — это достаточно. */
}

/* ── Инициализация micro-ROS (повторяется до успеха) ─────── */
static bool ros_init(void)
{
    extern bool usb_cdc_transport_open(struct uxrCustomTransport *t);
    extern bool usb_cdc_transport_close(struct uxrCustomTransport *t);
    extern size_t usb_cdc_transport_write(struct uxrCustomTransport *t,
        const uint8_t *buf, size_t len, uint8_t *err);
    extern size_t usb_cdc_transport_read(struct uxrCustomTransport *t,
        uint8_t *buf, size_t len, int timeout_ms, uint8_t *err);

    rmw_uros_set_custom_transport(
        true,
        NULL,
        usb_cdc_transport_open,
        usb_cdc_transport_close,
        usb_cdc_transport_write,
        usb_cdc_transport_read
    );

    g_allocator = rcl_get_default_allocator();

    if (rmw_uros_ping_agent(MICROROS_AGENT_TIMEOUT_MS,
                            MICROROS_AGENT_ATTEMPTS) != RMW_RET_OK)
        return false;

    if (rclc_support_init(&g_support, 0, NULL, &g_allocator) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&g_node, "d_grape", "", &g_support) != RCL_RET_OK)
        return false;

    rclc_publisher_init_default(
        &g_pub_imu, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/d_grape/imu/filtered");

    rclc_publisher_init_default(
        &g_pub_vel, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "/d_grape/velocity");

    rclc_publisher_init_default(
        &g_pub_diag, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
        "/d_grape/diagnostics");

    rclc_subscription_init_default(
        &g_sub_wheel_cmd, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "/d_grape/wheel_cmd");

    rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator);
    rclc_executor_add_subscription(
        &g_executor, &g_sub_wheel_cmd,
        &g_msg_wheel_cmd, wheel_cmd_callback, ON_NEW_DATA);

    return true;
}

#endif /* ifndef DEBUG_MODE */

/* ══════════════════════════════════════════════════════════ */
/*  ЗАДАЧА: IMU  500 Гц                                       */
/* ══════════════════════════════════════════════════════════ */
static void task_imu(void *arg)
{
    (void)arg;
    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(tick += TASK_IMU_PERIOD_MS);

        IMU_Raw_t raw;
        if (imu_read(&hi2c1, &raw) == HAL_OK) {
            osMutexAcquire(g_imu_mutex, osWaitForever);
            g_imu_raw = raw;
            osMutexRelease(g_imu_mutex);
        }
    }
}

/* ══════════════════════════════════════════════════════════ */
/*  ЗАДАЧА: ROBOT CONTROL  100 Гц                             */
/* ══════════════════════════════════════════════════════════ */
static void task_robot(void *arg)
{
    (void)arg;
    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(tick += TASK_ROBOT_PERIOD_MS);
        const float dt = PID_DT_S;

        /* 1. Энкодеры */
        encoder_update(&g_enc_left,  dt);
        encoder_update(&g_enc_right, dt);

        float v_left  = g_enc_left.speed_mps;
        float v_right = g_enc_right.speed_mps;
        float v_enc   = (v_left + v_right) * 0.5f;
        float omega_enc = (v_right - v_left) / ROBOT_TRACK_WIDTH;

        g_enc_left_mps  = v_left;
        g_enc_right_mps = v_right;

        /* 2. Читаем IMU */
        IMU_Raw_t imu;
        osMutexAcquire(g_imu_mutex, osWaitForever);
        imu = g_imu_raw;
        osMutexRelease(g_imu_mutex);

        /* 3. Фильтр Калмана */
        float vel_fused   = kf_vel_update(&g_kf_vel, imu.ax, v_enc);
        float omega_fused = kf_yaw_update(&g_kf_yaw, imu.gz, omega_enc,
                                          KF_YAW_R_GYRO, KF_YAW_R_ENC_W);

        osMutexAcquire(g_kf_mutex, osWaitForever);
        g_kf_out.velocity_mps = vel_fused;
        g_kf_out.omega_rads   = omega_fused;
        g_kf_out.accel_bias   = g_kf_vel.x[1];
        g_kf_out.gyro_bias    = g_kf_yaw.x[1];
        g_kf_out.valid        = imu.valid;
        osMutexRelease(g_kf_mutex);

        /* 4. Safety timeout — читаем g_last_cmd_ms атомарно */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint32_t last_cmd = g_last_cmd_ms;
        __set_PRIMASK(primask);

        if ((osKernelGetTickCount() - last_cmd) > CMD_TIMEOUT_MS) {
            primask = __get_PRIMASK();
            __disable_irq();
            g_cmd_left_mps  = 0.0f;
            g_cmd_right_mps = 0.0f;
            __set_PRIMASK(primask);
        }

        /* 5. PID velocity control — читаем команды атомарно */
        primask = __get_PRIMASK();
        __disable_irq();
        float cmd_left  = g_cmd_left_mps;
        float cmd_right = g_cmd_right_mps;
        __set_PRIMASK(primask);

        motor_velocity_update(&g_motor_left,  cmd_left,  dt);
        motor_velocity_update(&g_motor_right, cmd_right, dt);

        /* 6. Watchdog ping */
        osThreadFlagsSet(g_watchdog_handle, 0x02U);
    }
}

/* ══════════════════════════════════════════════════════════ */
/*  ЗАДАЧА: micro-ROS  40 Гц  (не компилируется в DEBUG_MODE) */
/* ══════════════════════════════════════════════════════════ */
#ifndef DEBUG_MODE
static void task_microros(void *arg)
{
    (void)arg;

    while (!ros_init()) {
        osDelay(1000);
    }

    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(tick += TASK_MICROROS_PERIOD_MS);

        /* Получаем wheel_cmd */
        rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));

        /* Читаем KF output */
        KF_Output_t kf;
        osMutexAcquire(g_kf_mutex, osWaitForever);
        kf = g_kf_out;
        osMutexRelease(g_kf_mutex);

        if (!kf.valid) {
            RCL_PUB(&g_pub_diag, &g_msg_diag);
            continue;
        }

        g_msg_imu.linear_acceleration.x            = kf.velocity_mps;
        g_msg_imu.linear_acceleration.y            = 0.0f;
        g_msg_imu.linear_acceleration.z            = 0.0f;
        g_msg_imu.angular_velocity.x               = 0.0f;
        g_msg_imu.angular_velocity.y               = 0.0f;
        g_msg_imu.angular_velocity.z               = kf.omega_rads;
        g_msg_imu.linear_acceleration_covariance[0] = g_kf_vel.P[0][0];
        g_msg_imu.angular_velocity_covariance[8]    = g_kf_yaw.P[0][0];
        RCL_PUB(&g_pub_imu, &g_msg_imu);

        g_msg_vel.x = kf.velocity_mps;
        g_msg_vel.y = 0.0f;
        g_msg_vel.z = kf.omega_rads;
        RCL_PUB(&g_pub_vel, &g_msg_vel);

        RCL_PUB(&g_pub_diag, &g_msg_diag);
    }
}
#endif /* ifndef DEBUG_MODE */

/* ══════════════════════════════════════════════════════════ */
/*  ЗАДАЧА: WATCHDOG  high priority                           */
/* ══════════════════════════════════════════════════════════ */
static void task_watchdog(void *arg)
{
    (void)arg;

    for (;;) {
        uint32_t flags = osThreadFlagsWait(0x02U, osFlagsWaitAny, 50U);

        if (flags == osFlagsErrorTimeout) {
            /* task_robot завис — аварийная остановка и ребут */
            motor_stop(&g_motor_left);
            motor_stop(&g_motor_right);
            motor_stop(&g_motor_skis);
            osDelay(100);
            HAL_NVIC_SystemReset();
        }
    }
}

/* ══════════════════════════════════════════════════════════ */
/*  ИНИЦИАЛИЗАЦИЯ АППАРАТУРЫ                                  */
/* ══════════════════════════════════════════════════════════ */
static void hw_init(void)
{
    encoder_init(&g_enc_left);
    encoder_init(&g_enc_right);

    motor_init(&g_motor_left,  &g_enc_left,  PID_DT_S);
    motor_init(&g_motor_right, &g_enc_right, PID_DT_S);
    motor_init(&g_motor_skis,  NULL,         PID_DT_S);

    /* IMU: ошибка не фатальна — KF работает без IMU по энкодерам */
    imu_init(&hi2c1);

    kf_vel_init(&g_kf_vel, PID_DT_S,
                KF_VEL_Q_V, KF_VEL_Q_BA, KF_VEL_R_ENC);

    kf_yaw_init(&g_kf_yaw, PID_DT_S,
                KF_YAW_Q_W, KF_YAW_Q_BG, KF_YAW_R_ENC_W);

    pid_enable(&g_motor_left.pid);
    pid_enable(&g_motor_right.pid);

    MX_USB_DEVICE_Init();
}

/* ══════════════════════════════════════════════════════════ */
/*  ЗАДАЧА: DEBUG  10 Гц  (только при DEBUG_MODE)             */
/* ══════════════════════════════════════════════════════════ */
#ifdef DEBUG_MODE
static void task_debug(void *arg)
{
    (void)arg;

    debug_log("\r\n*** D-Grape DEBUG MODE — micro-ROS disabled ***\r\n");

    uint32_t tick = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(tick += TASK_DEBUG_PERIOD_MS);

        DebugSnapshot_t snap = {0};

        /* Временная метка */
        snap.tick_ms = HAL_GetTick();

        /* IMU — под мьютексом */
        osMutexAcquire(g_imu_mutex, osWaitForever);
        IMU_Raw_t imu = g_imu_raw;
        osMutexRelease(g_imu_mutex);

        snap.ax        = imu.ax;
        snap.ay        = imu.ay;
        snap.az        = imu.az;
        snap.gx        = imu.gx;
        snap.gy        = imu.gy;
        snap.gz        = imu.gz;
        snap.temp_c    = imu.temp_c;
        snap.imu_valid = imu.valid;

        /* Энкодеры — volatile, читаем атомарно */
        snap.enc_left_mps    = g_enc_left_mps;
        snap.enc_right_mps   = g_enc_right_mps;
        snap.enc_left_dist_m  = g_enc_left.distance_m;
        snap.enc_right_dist_m = g_enc_right.distance_m;

        /* Kalman — под мьютексом */
        osMutexAcquire(g_kf_mutex, osWaitForever);
        KF_Output_t kf = g_kf_out;
        osMutexRelease(g_kf_mutex);

        snap.kf_vel_mps    = kf.velocity_mps;
        snap.kf_omega_rads = kf.omega_rads;
        snap.kf_accel_bias = kf.accel_bias;
        snap.kf_gyro_bias  = kf.gyro_bias;
        snap.kf_p00_vel    = g_kf_vel.P[0][0];
        snap.kf_p00_yaw    = g_kf_yaw.P[0][0];

        /* PID */
        snap.pid_setpoint_left  = g_motor_left.pid.setpoint;
        snap.pid_setpoint_right = g_motor_right.pid.setpoint;
        snap.pid_out_left       = g_motor_left.pid.output;
        snap.pid_out_right      = g_motor_right.pid.output;
        snap.pid_integral_left  = g_motor_left.pid.integral;
        snap.pid_integral_right = g_motor_right.pid.integral;

        /* Команда — critical section */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        snap.cmd_left_mps  = g_cmd_left_mps;
        snap.cmd_right_mps = g_cmd_right_mps;
        snap.last_cmd_ms   = g_last_cmd_ms;
        __set_PRIMASK(primask);

        debug_print(&snap);
    }
}
#endif /* DEBUG_MODE */

/* ══════════════════════════════════════════════════════════ */
/*  ТОЧКА ВХОДА                                               */
/* ══════════════════════════════════════════════════════════ */
void freertos_app_init(void)
{
    hw_init();

    g_imu_mutex = osMutexNew(NULL);
    g_kf_mutex  = osMutexNew(NULL);

#ifdef DEBUG_MODE
    debug_init();
    console_init();
#endif

    osThreadAttr_t attr = {0};

    attr.name       = "imu";
    attr.stack_size = TASK_IMU_STACK;
    attr.priority   = TASK_IMU_PRIORITY;
    osThreadNew(task_imu, NULL, &attr);

    attr.name       = "robot";
    attr.stack_size = TASK_ROBOT_STACK;
    attr.priority   = TASK_ROBOT_PRIORITY;
    osThreadNew(task_robot, NULL, &attr);

#ifdef DEBUG_MODE
    /* В отладочном режиме — task_debug + task_console вместо task_microros */
    attr.name       = "debug";
    attr.stack_size = TASK_DEBUG_STACK;
    attr.priority   = TASK_DEBUG_PRIORITY;
    osThreadNew(task_debug, NULL, &attr);

    attr.name       = "console";
    attr.stack_size = TASK_CONSOLE_STACK;
    attr.priority   = TASK_CONSOLE_PRIORITY;
    osThreadNew(task_console, NULL, &attr);
#else
    attr.name       = "microros";
    attr.stack_size = TASK_MICROROS_STACK;
    attr.priority   = TASK_MICROROS_PRIORITY;
    osThreadNew(task_microros, NULL, &attr);
#endif

    attr.name       = "watchdog";
    attr.stack_size = TASK_WATCHDOG_STACK;
    attr.priority   = TASK_WATCHDOG_PRIORITY;
    g_watchdog_handle = osThreadNew(task_watchdog, NULL, &attr);
}