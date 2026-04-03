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
#  include <geometry_msgs/msg/twist.h>
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

static osThreadId_t      g_watchdog_handle;

/* ══════════════════════════════════════════════════════════ */
/*  АППАРАТНЫЕ ОБЪЕКТЫ                                        */
/* ══════════════════════════════════════════════════════════ */

/* Энкодеры не подключены — структуры нулевые.
 * Когда подключат: g_enc_left = { .htim = &htim3 } (TIM3, PA6/PA7),
 *                  g_enc_right = { .htim = &htim4 } (TIM4, PB6/PB7),
 *                  перенести левый LPWM с PA7 на PB10 (TIM2_CH3). */
Encoder_t g_enc_left  = {0};
Encoder_t g_enc_right = {0};

/* Моторы инициализируются runtime в hw_init() —
 * статический инициализатор не может использовать адреса регистров. */
Motor_t g_motor_left  = {0};
Motor_t g_motor_right = {0};
Motor_t g_motor_skis  = {0};

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
static geometry_msgs__msg__Twist             g_msg_wheel_cmd;

/* ── Callback: cmd_vel (geometry_msgs/Twist → left/right m/s) ─ */
static void wheel_cmd_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *cmd =
        (const geometry_msgs__msg__Twist *)msg_in;

    /* Дифференциальная кинематика:
     *   v_l = vx - wz * (track/2)
     *   v_r = vx + wz * (track/2)
     * ROBOT_TRACK_WIDTH = 0.526 м → half = 0.263 м */
    const float half_track = ROBOT_TRACK_WIDTH * 0.5f;
    float vx = cmd->linear.x;
    float wz = cmd->angular.z;

    if (!isfinite(vx)) vx = 0.0f;
    if (!isfinite(wz)) wz = 0.0f;

    float l = vx - wz * half_track;
    float r = vx + wz * half_track;

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
static bool s_ros_initialized = false;

static bool ros_init(void)
{
    if (s_ros_initialized) {
        rclc_executor_fini(&g_executor);
        rcl_subscription_fini(&g_sub_wheel_cmd, &g_node);
        rcl_publisher_fini(&g_pub_diag, &g_node);
        rcl_publisher_fini(&g_pub_vel, &g_node);
        rcl_publisher_fini(&g_pub_imu, &g_node);
        rcl_node_fini(&g_node);
        rclc_support_fini(&g_support);
        s_ros_initialized = false;
    }

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

    if (rclc_node_init_default(&g_node, "microros_hardware", "d_grape", &g_support) != RCL_RET_OK) {
        rclc_support_fini(&g_support);
        return false;
    }

    /* Даём XRCE-DDS выслать pending ACK/heartbeat после node_init
     * до того как начнём создавать publishers. Без этой паузы
     * pending ACK + CREATE_TOPIC пакуются в один PDU и могут
     * превысить MTU или застать CDC TX занятым. */
    rmw_uros_ping_agent(50, 1);

    /* Относительные имена топиков — пространство имён ноды (/d_grape)
     * подставляется автоматически: "imu/filtered" → /d_grape/imu/filtered */
    if (rclc_publisher_init_default(&g_pub_imu, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/filtered") != RCL_RET_OK) {
        rcl_node_fini(&g_node); rclc_support_fini(&g_support); return false;
    }

    if (rclc_publisher_init_default(&g_pub_vel, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "velocity") != RCL_RET_OK) {
        rcl_publisher_fini(&g_pub_imu, &g_node);
        rcl_node_fini(&g_node); rclc_support_fini(&g_support); return false;
    }

    if (rclc_publisher_init_default(&g_pub_diag, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
        "diagnostics") != RCL_RET_OK) {
        rcl_publisher_fini(&g_pub_vel, &g_node);
        rcl_publisher_fini(&g_pub_imu, &g_node);
        rcl_node_fini(&g_node); rclc_support_fini(&g_support); return false;
    }

    if (rclc_subscription_init_default(&g_sub_wheel_cmd, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) {
        rcl_publisher_fini(&g_pub_diag, &g_node);
        rcl_publisher_fini(&g_pub_vel, &g_node);
        rcl_publisher_fini(&g_pub_imu, &g_node);
        rcl_node_fini(&g_node); rclc_support_fini(&g_support); return false;
    }

    if (rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator) != RCL_RET_OK) {
        rcl_subscription_fini(&g_sub_wheel_cmd, &g_node);
        rcl_publisher_fini(&g_pub_diag, &g_node);
        rcl_publisher_fini(&g_pub_vel, &g_node);
        rcl_publisher_fini(&g_pub_imu, &g_node);
        rcl_node_fini(&g_node); rclc_support_fini(&g_support); return false;
    }

    rclc_executor_add_subscription(
        &g_executor, &g_sub_wheel_cmd,
        &g_msg_wheel_cmd, wheel_cmd_callback, ON_NEW_DATA);

    s_ros_initialized = true;
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
    uint16_t imu_err_count = 0U;

    /* Инициализируем IMU здесь — после старта планировщика.
     * HAL_Delay внутри imu_init работает корректно только когда
     * SysTick не захвачен FreeRTOS до osKernelStart. */
    imu_init(&hi2c1);

    /* LD4 зелёный мигает медленно пока IMU не ответил */
    HAL_GPIO_WritePin(LED_IMU_PORT, LED_IMU_PIN, GPIO_PIN_RESET);

    /* Сбрасываем tick после imu_init: он занимает ~110 мс (osDelay внутри),
     * иначе osDelayUntil обнаружит что время уже прошло и вернёт ошибку
     * без блокировки — task_imu начнёт крутиться без yield, вытесняя
     * task_robot и не давая ему пинговать watchdog. */
    tick = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(tick += TASK_IMU_PERIOD_MS);

        IMU_Raw_t raw;
        if (imu_read(&hi2c1, &raw) == HAL_OK) {
            osMutexAcquire(g_imu_mutex, osWaitForever);
            g_imu_raw = raw;
            osMutexRelease(g_imu_mutex);
            imu_err_count = 0U;
            HAL_GPIO_WritePin(LED_IMU_PORT, LED_IMU_PIN, GPIO_PIN_SET);
        } else {
            imu_err_count++;
            /* Мигаем раз в 500 мс = 250 тиков по 2 мс */
            if (imu_err_count >= 250U) {
                HAL_GPIO_TogglePin(LED_IMU_PORT, LED_IMU_PIN);
                imu_err_count = 0U;
            }
            /* I2C завис на таймауте (~2 мс polling busy-wait при IMU_I2C_TIMEOUT_MS=1).
             * Если не сбросить tick, osDelayUntil вернётся сразу (цель уже в прошлом),
             * task_imu начнёт крутиться без yield и вытеснит task_robot → watchdog reset. */
            tick = osKernelGetTickCount();
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

        /* 1. Энкодеры не подключены — скорости нулевые */

        /* 2. Читаем IMU */
        IMU_Raw_t imu;
        osMutexAcquire(g_imu_mutex, osWaitForever);
        imu = g_imu_raw;
        osMutexRelease(g_imu_mutex);

        /* 3. Фильтр Калмана — predict only (нет энкодеров) */
        float vel_fused   = kf_vel_predict_only(&g_kf_vel, imu.ax);
        float omega_fused = kf_yaw_predict_only(&g_kf_yaw, imu.gz);

        osMutexAcquire(g_kf_mutex, osWaitForever);
        g_kf_out.velocity_mps = vel_fused;
        g_kf_out.omega_rads   = omega_fused;
        g_kf_out.accel_bias   = g_kf_vel.x[1];
        g_kf_out.gyro_bias    = g_kf_yaw.x[1];
        g_kf_out.kf_p00_vel   = g_kf_vel.P[0][0];
        g_kf_out.kf_p00_yaw   = g_kf_yaw.P[0][0];
        g_kf_out.valid        = imu.valid;
        osMutexRelease(g_kf_mutex);

        /* 4. Safety timeout */
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

        /* 5. PID */
        primask = __get_PRIMASK();
        __disable_irq();
        float cmd_left  = g_cmd_left_mps;
        float cmd_right = g_cmd_right_mps;
        __set_PRIMASK(primask);

        motor_velocity_update(&g_motor_left,  cmd_left,   dt);
        motor_velocity_update(&g_motor_right, -cmd_right, dt); /* правый мотор стоит инвертированно */

        /* 6. LED */
        static uint8_t hb_cnt = 0U;
        if (++hb_cnt >= 50U) {
            HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
            hb_cnt = 0U;
        }
        HAL_GPIO_WritePin(LED_MOTORS_PORT, LED_MOTORS_PIN,
            (cmd_left != 0.0f || cmd_right != 0.0f)
                ? GPIO_PIN_SET : GPIO_PIN_RESET);

        /* 7. Watchdog ping */
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

    /* Инициализируем USB CDC — без этого transport не работает.
     * Вызов здесь (после старта планировщика) обязателен: стек USB
     * использует прерывания, которые должны быть настроены до
     * первого обращения к USB OTG FS. */
    MX_USB_DEVICE_Init();

    /* Даём хосту время перечислить устройство (CDC enumeration) */
    osDelay(1000);

    /* LD6 синий — мигаем пока ищем агента */
    HAL_GPIO_WritePin(LED_COMM_PORT, LED_COMM_PIN, GPIO_PIN_RESET);

    while (!ros_init()) {
        HAL_GPIO_TogglePin(LED_COMM_PORT, LED_COMM_PIN);
        osDelay(500);
    }

    /* Агент найден — LD6 горит постоянно */
    HAL_GPIO_WritePin(LED_COMM_PORT, LED_COMM_PIN, GPIO_PIN_SET);

    uint32_t tick = osKernelGetTickCount();
    uint16_t ping_cnt = 0U;

    for (;;) {
        osDelayUntil(tick += TASK_MICROROS_PERIOD_MS);

        /* Раз в ~2 с проверяем связь с агентом */
        if (++ping_cnt >= (1000U / TASK_MICROROS_PERIOD_MS)) {
            ping_cnt = 0U;
            if (rmw_uros_ping_agent(MICROROS_AGENT_TIMEOUT_MS,
                                    MICROROS_AGENT_ATTEMPTS) != RMW_RET_OK) {
                HAL_GPIO_WritePin(LED_COMM_PORT, LED_COMM_PIN, GPIO_PIN_RESET);
                while (!ros_init()) {
                    HAL_GPIO_TogglePin(LED_COMM_PORT, LED_COMM_PIN);
                    osDelay(100);
                }
                HAL_GPIO_WritePin(LED_COMM_PORT, LED_COMM_PIN, GPIO_PIN_SET);
                tick = osKernelGetTickCount();
                ping_cnt = 0U;
                continue;
            }
        }

        rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));

        KF_Output_t kf;
        osMutexAcquire(g_kf_mutex, osWaitForever);
        kf = g_kf_out;
        osMutexRelease(g_kf_mutex);

        g_msg_imu.linear_acceleration.x            = kf.velocity_mps;
        g_msg_imu.linear_acceleration.y            = 0.0f;
        g_msg_imu.linear_acceleration.z            = 0.0f;
        g_msg_imu.angular_velocity.x               = 0.0f;
        g_msg_imu.angular_velocity.y               = 0.0f;
        g_msg_imu.angular_velocity.z               = kf.omega_rads;
        g_msg_imu.linear_acceleration_covariance[0] = kf.kf_p00_vel;
        g_msg_imu.angular_velocity_covariance[8]    = kf.kf_p00_yaw;
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
        uint32_t flags = osThreadFlagsWait(0x02U, osFlagsWaitAny, 200U);

        if (flags == osFlagsErrorTimeout) {
            /* task_robot завис — LD7 красный, аварийная остановка, ребут */
            HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET);
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
    /* ── Моторы BTS7960B: runtime инициализация ─────────────*/
    /* Левый: PA15(TIM2_CH1)=RPWM, PA7(TIM3_CH2)=LPWM */
    g_motor_left.htim_fwd = &htim2;  g_motor_left.ch_fwd  = MOTOR_LEFT_CH_FWD;
    g_motor_left.ccr_fwd  = &MOTOR_LEFT_CCR_FWD;
    g_motor_left.htim_rev = &htim3;  g_motor_left.ch_rev  = MOTOR_LEFT_CH_REV;
    g_motor_left.ccr_rev  = &MOTOR_LEFT_CCR_REV;
    g_motor_left.arr      = &TIM2->ARR;

    /* Правый: PB3(TIM2_CH2)=RPWM, PB11(TIM2_CH4)=LPWM */
    g_motor_right.htim_fwd = &htim2;  g_motor_right.ch_fwd  = MOTOR_RIGHT_CH_FWD;
    g_motor_right.ccr_fwd  = &MOTOR_RIGHT_CCR_FWD;
    g_motor_right.htim_rev = &htim2;  g_motor_right.ch_rev  = MOTOR_RIGHT_CH_REV;
    g_motor_right.ccr_rev  = &MOTOR_RIGHT_CCR_REV;
    g_motor_right.arr      = &TIM2->ARR;

    /* Лыжи: TIM1_CH1 (одиночный ШИМ, REV = заглушка на тот же CCR) */
    g_motor_skis.htim_fwd = &htim1;  g_motor_skis.ch_fwd  = MOTOR_SKIS_CHANNEL;
    g_motor_skis.ccr_fwd  = &MOTOR_SKIS_CCR;
    g_motor_skis.htim_rev = &htim1;  g_motor_skis.ch_rev  = MOTOR_SKIS_CHANNEL;
    g_motor_skis.ccr_rev  = &MOTOR_SKIS_CCR;
    g_motor_skis.arr      = &TIM1->ARR;

    motor_init(&g_motor_left,  NULL, PID_DT_S);   /* enc=NULL: открытый контур */
    motor_init(&g_motor_right, NULL, PID_DT_S);
    motor_init(&g_motor_skis,  NULL, PID_DT_S);

    /* Энкодеры не подключены — encoder_init не вызываем.
     * Восстановить когда энкодеры будут подключены:
     *   encoder_init(&g_enc_left);
     *   encoder_init(&g_enc_right);
     *   motor_init(&g_motor_left,  &g_enc_left,  PID_DT_S);
     *   motor_init(&g_motor_right, &g_enc_right, PID_DT_S); */

    /* imu_init(&hi2c1); — отключено: HAL_Delay зависает до старта планировщика */

    kf_vel_init(&g_kf_vel, PID_DT_S,
                KF_VEL_Q_V, KF_VEL_Q_BA, KF_VEL_R_ENC);

    kf_yaw_init(&g_kf_yaw, PID_DT_S,
                KF_YAW_Q_W, KF_YAW_Q_BG, KF_YAW_R_ENC_W);
    /* imu_init вызывается внутри task_imu после старта планировщика */
}

/* ══════════════════════════════════════════════════════════ */
/*  ЗАДАЧА: DEBUG  10 Гц  (только при DEBUG_MODE)             */
/* ══════════════════════════════════════════════════════════ */
#ifdef DEBUG_MODE
static void task_debug(void *arg)
{
    (void)arg;

    /* ── Шаг 1: инициализация USB CDC ──────────────────────
     * В DEBUG_MODE task_microros не запускается, поэтому
     * MX_USB_DEVICE_Init() нужно вызвать здесь. */
    MX_USB_DEVICE_Init();

    /* ── Шаг 2: ждём enumeration хостом ────────────────────
     * Мигаем синим LED пока хост не сконфигурировал устройство.
     * Если за 5 секунд не появилось — идём дальше всё равно. */
    {
        uint32_t start = HAL_GetTick();
        uint32_t blink = start;
        while ((HAL_GetTick() - start) < 5000U) {
            if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
                break;
            if (HAL_GetTick() - blink >= 200U) {
                HAL_GPIO_TogglePin(LED_COMM_PORT, LED_COMM_PIN);
                blink = HAL_GetTick();
            }
            osDelay(10);
        }
    }

    /* LD6 синий — горит постоянно если USB OK, мигает если нет */
    HAL_GPIO_WritePin(LED_COMM_PORT, LED_COMM_PIN,
        (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
            ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Пауза — хост открывает COM-порт */
    osDelay(200);

    /* ── Шаг 3: стартовый баннер ────────────────────── */
    debug_print_startup();

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

        /* Энкодеры не подключены */
        snap.enc_left_mps    = 0.0f;
        snap.enc_right_mps   = 0.0f;
        snap.enc_left_dist_m  = 0.0f;
        snap.enc_right_dist_m = 0.0f;

        /* Kalman — под мьютексом */
        osMutexAcquire(g_kf_mutex, osWaitForever);
        KF_Output_t kf = g_kf_out;
        osMutexRelease(g_kf_mutex);

        snap.kf_vel_mps    = kf.velocity_mps;
        snap.kf_omega_rads = kf.omega_rads;
        snap.kf_accel_bias = kf.accel_bias;
        snap.kf_gyro_bias  = kf.gyro_bias;
        snap.kf_p00_vel    = kf.kf_p00_vel;
        snap.kf_p00_yaw    = kf.kf_p00_yaw;

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

        /* USB CDC статус */
        snap.usb_configured = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
        snap.usb_tx_errors  = s_tx_errors_debug;
        snap.usb_rx_total   = rx_total_debug;

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

    /* Если мьютекс не создан — heap исчерпан до старта FreeRTOS.
     * В этом случае vApplicationMallocFailedHook не вызывается,
     * поэтому явно входим в fault-петлю (PD14 + PE1 мигают). */
    if (!g_imu_mutex || !g_kf_mutex) {
        while (1) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
            for (volatile uint32_t i = 0; i < 8400000UL; i++) {}
        }
    }

#ifdef DEBUG_MODE
    debug_init();
    console_init();
#endif

    osThreadAttr_t attr = {0};
    osThreadId_t   tid  = NULL;

    attr.name       = "imu";
    attr.stack_size = TASK_IMU_STACK;
    attr.priority   = TASK_IMU_PRIORITY;
    tid = osThreadNew(task_imu, NULL, &attr);
    if (!tid) goto task_create_failed;

    attr.name       = "robot";
    attr.stack_size = TASK_ROBOT_STACK;
    attr.priority   = TASK_ROBOT_PRIORITY;
    tid = osThreadNew(task_robot, NULL, &attr);
    if (!tid) goto task_create_failed;

#ifdef DEBUG_MODE
    attr.name       = "debug";
    attr.stack_size = TASK_DEBUG_STACK;
    attr.priority   = TASK_DEBUG_PRIORITY;
    tid = osThreadNew(task_debug, NULL, &attr);
    if (!tid) goto task_create_failed;

    attr.name       = "console";
    attr.stack_size = TASK_CONSOLE_STACK;
    attr.priority   = TASK_CONSOLE_PRIORITY;
    tid = osThreadNew(task_console, NULL, &attr);
    if (!tid) goto task_create_failed;
#else
    attr.name       = "microros";
    attr.stack_size = TASK_MICROROS_STACK;
    attr.priority   = TASK_MICROROS_PRIORITY;
    tid = osThreadNew(task_microros, NULL, &attr);
    if (!tid) goto task_create_failed;
#endif

    attr.name       = "watchdog";
    attr.stack_size = TASK_WATCHDOG_STACK;
    attr.priority   = TASK_WATCHDOG_PRIORITY;
    g_watchdog_handle = osThreadNew(task_watchdog, NULL, &attr);
    if (!g_watchdog_handle) goto task_create_failed;

    return;

task_create_failed:
    while (1) {
        HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
        HAL_GPIO_TogglePin(LED_ERROR_PORT,     LED_ERROR_PIN);
        for (volatile uint32_t i = 0; i < 400000UL; i++) {}
    }
}