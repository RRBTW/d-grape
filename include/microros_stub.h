#pragma once
/* ============================================================
 * Заглушки micro-ROS — временно, пока не подключена библиотека
 * Позволяют скомпилировать проект и проверить остальной код
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>

/* rcl types */
typedef struct { void *impl; } rcl_node_t;
typedef struct { void *impl; } rcl_publisher_t;
typedef struct { void *impl; } rcl_subscription_t;
typedef struct { void *impl; } rcl_allocator_t;
typedef struct { void *impl; void *context; } rclc_support_t;
typedef struct { void *impl; } rclc_executor_t;
typedef int rcl_ret_t;
#define RCL_RET_OK    0
#define RCL_MS_TO_NS(ms) ((ms) * 1000000ULL)
#define ON_NEW_DATA   0
#define ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, sub, name) NULL

/* rmw */
typedef int rmw_ret_t;
#define RMW_RET_OK 0
static inline rmw_ret_t rmw_uros_ping_agent(int ms, int attempts)
    { (void)ms; (void)attempts; return RMW_RET_OK; }

/* rclc */
static inline rcl_ret_t rclc_support_init(rclc_support_t *s, int argc,
    char **argv, rcl_allocator_t *a)
    { (void)s;(void)argc;(void)argv;(void)a; return RCL_RET_OK; }
static inline rcl_ret_t rclc_node_init_default(rcl_node_t *n,
    const char *name, const char *ns, rclc_support_t *s)
    { (void)n;(void)name;(void)ns;(void)s; return RCL_RET_OK; }
static inline rcl_ret_t rclc_publisher_init_default(rcl_publisher_t *p,
    rcl_node_t *n, const void *ts, const char *topic)
    { (void)p;(void)n;(void)ts;(void)topic; return RCL_RET_OK; }
static inline rcl_ret_t rclc_subscription_init_default(rcl_subscription_t *s,
    rcl_node_t *n, const void *ts, const char *topic)
    { (void)s;(void)n;(void)ts;(void)topic; return RCL_RET_OK; }
static inline rcl_ret_t rclc_executor_init(rclc_executor_t *e,
    void *ctx, int n, rcl_allocator_t *a)
    { (void)e;(void)ctx;(void)n;(void)a; return RCL_RET_OK; }
static inline rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *e,
    rcl_subscription_t *s, void *msg, void (*cb)(const void*), int mode)
    { (void)e;(void)s;(void)msg;(void)cb;(void)mode; return RCL_RET_OK; }
static inline rcl_ret_t rclc_executor_spin_some(rclc_executor_t *e, uint64_t ns)
    { (void)e;(void)ns; return RCL_RET_OK; }
static inline rcl_ret_t rcl_publish(rcl_publisher_t *p,
    const void *msg, void *a)
    { (void)p;(void)msg;(void)a; return RCL_RET_OK; }
static inline rcl_allocator_t rcl_get_default_allocator(void)
    { rcl_allocator_t a = {0}; return a; }

/* Message types — минимальные заглушки */
typedef struct { double x, y, z; }    Vector3_stub;
typedef struct {
    struct { double x,y,z; } linear_acceleration;
    struct { double x,y,z; } angular_velocity;
    double linear_acceleration_covariance[9];
    double angular_velocity_covariance[9];
} Imu_stub;
typedef struct { int level; }          DiagnosticArray_stub;

#define geometry_msgs__msg__Vector3           Vector3_stub
#define sensor_msgs__msg__Imu                 Imu_stub
#define diagnostic_msgs__msg__DiagnosticArray DiagnosticArray_stub