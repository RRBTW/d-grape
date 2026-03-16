/* ============================================================
 *  console.c — Интерактивная консоль управления по USB CDC
 * ============================================================ */

#include "console.h"

#ifdef DEBUG_MODE

#include "freertos_app.h"   /* extern Motor_t, Encoder_t, g_cmd_*, g_last_cmd_ms */
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* rx_ring и указатели объявлены в microros_transport.c без static */
extern uint8_t           rx_ring[512U];
extern volatile uint32_t rx_head;
extern volatile uint32_t rx_tail;

/* ── Внутреннее состояние консоли ───────────────────────────*/
static char     s_line[CONSOLE_LINE_BUF]; /* буфер набираемой строки */
static uint8_t  s_line_len = 0U;

/* Текущая уставка wasd (чтобы накапливать шаги) */
static float s_wasd_left  = 0.0f;
static float s_wasd_right = 0.0f;

/* ── Вспомогательные функции ────────────────────────────────*/

/* Отправить строку в USB CDC с простым polling */
static void con_puts(const char *str)
{
    if (!str) return;
    uint16_t len = (uint16_t)strnlen(str, 256U);
    uint32_t deadline = HAL_GetTick() + 30U;
    while (HAL_GetTick() < deadline) {
        if (CDC_Transmit_FS((uint8_t *)str, len) == USBD_OK)
            return;
        osDelay(1);
    }
}

/* Отправить один символ (эхо) */
static void con_echo(char c)
{
    uint8_t b = (uint8_t)c;
    uint32_t deadline = HAL_GetTick() + 10U;
    while (HAL_GetTick() < deadline) {
        if (CDC_Transmit_FS(&b, 1U) == USBD_OK)
            return;
        osDelay(1);
    }
}

/* Применить уставку скоростей (thread-safe) */
void console_set_cmd(float left_mps, float right_mps)
{
    /* Clamp */
    if (left_mps  >  CONSOLE_MAX_MPS) left_mps  =  CONSOLE_MAX_MPS;
    if (left_mps  < -CONSOLE_MAX_MPS) left_mps  = -CONSOLE_MAX_MPS;
    if (right_mps >  CONSOLE_MAX_MPS) right_mps =  CONSOLE_MAX_MPS;
    if (right_mps < -CONSOLE_MAX_MPS) right_mps = -CONSOLE_MAX_MPS;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_cmd_left_mps  = left_mps;
    g_cmd_right_mps = right_mps;
    g_last_cmd_ms   = osKernelGetTickCount();
    __set_PRIMASK(primask);
}

/* ── Обработка одиночной клавиши (без Enter) ────────────────*/
static void handle_key(char key)
{
    char reply[48];

    switch (key) {
    case 'w':
        s_wasd_left  += CONSOLE_STEP_MPS;
        s_wasd_right += CONSOLE_STEP_MPS;
        console_set_cmd(s_wasd_left, s_wasd_right);
        snprintf(reply, sizeof(reply),
                 "\r\n> FWD  L=%.2f R=%.2f m/s\r\n",
                 (double)s_wasd_left, (double)s_wasd_right);
        con_puts(reply);
        break;

    case 's':
        s_wasd_left  -= CONSOLE_STEP_MPS;
        s_wasd_right -= CONSOLE_STEP_MPS;
        console_set_cmd(s_wasd_left, s_wasd_right);
        snprintf(reply, sizeof(reply),
                 "\r\n> BWD  L=%.2f R=%.2f m/s\r\n",
                 (double)s_wasd_left, (double)s_wasd_right);
        con_puts(reply);
        break;

    case 'a':
        s_wasd_left  -= CONSOLE_STEP_MPS;
        s_wasd_right += CONSOLE_STEP_MPS;
        console_set_cmd(s_wasd_left, s_wasd_right);
        snprintf(reply, sizeof(reply),
                 "\r\n> LEFT L=%.2f R=%.2f m/s\r\n",
                 (double)s_wasd_left, (double)s_wasd_right);
        con_puts(reply);
        break;

    case 'd':
        s_wasd_left  += CONSOLE_STEP_MPS;
        s_wasd_right -= CONSOLE_STEP_MPS;
        console_set_cmd(s_wasd_left, s_wasd_right);
        snprintf(reply, sizeof(reply),
                 "\r\n> RIGHT L=%.2f R=%.2f m/s\r\n",
                 (double)s_wasd_left, (double)s_wasd_right);
        con_puts(reply);
        break;

    case ' ':
        s_wasd_left  = 0.0f;
        s_wasd_right = 0.0f;
        console_set_cmd(0.0f, 0.0f);
        con_puts("\r\n> STOP\r\n");
        break;

    default:
        break; /* остальные символы уйдут в line-буфер */
    }
}

/* ── Парсинг и выполнение текстовой команды ─────────────────*/
static void handle_line(char *line)
{
    char reply[96];

    /* Убираем пробелы в начале */
    while (*line == ' ') line++;

    /* ── stop ─────────────────────────────────────────────── */
    if (strcmp(line, "stop") == 0) {
        s_wasd_left = s_wasd_right = 0.0f;
        console_set_cmd(0.0f, 0.0f);
        con_puts("> STOP\r\n");
        return;
    }

    /* ── reset ────────────────────────────────────────────── */
    if (strcmp(line, "reset") == 0) {
        encoder_reset(&g_enc_left);
        encoder_reset(&g_enc_right);
        con_puts("> encoders reset\r\n");
        return;
    }

    /* ── help ─────────────────────────────────────────────── */
    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        con_puts(
            "\r\n--- D-Grape console commands ---\r\n"
            "  w / s / a / d    forward / back / left / right (+0.1 m/s step)\r\n"
            "  [space]          stop immediately\r\n"
            "  left <v> right <v>   set wheel speeds [m/s], e.g.: left 0.3 right 0.3\r\n"
            "  stop             stop + reset PID integrators\r\n"
            "  reset            zero encoders\r\n"
            "  pid kp <v>       set PID Kp (both motors)\r\n"
            "  pid ki <v>       set PID Ki\r\n"
            "  pid kd <v>       set PID Kd\r\n"
            "  pid show         print current PID gains\r\n"
            "  help / ?         this message\r\n"
            "--------------------------------\r\n"
        );
        return;
    }

    /* ── left <v> right <v> ───────────────────────────────── */
    float lv = 0.0f, rv = 0.0f;
    if (sscanf(line, "left %f right %f", &lv, &rv) == 2) {
        s_wasd_left  = lv;
        s_wasd_right = rv;
        console_set_cmd(lv, rv);
        snprintf(reply, sizeof(reply),
                 "> cmd  L=%.3f  R=%.3f m/s\r\n",
                 (double)lv, (double)rv);
        con_puts(reply);
        return;
    }

    /* ── pid kp/ki/kd <v> ────────────────────────────────── */
    char  pid_param[8] = {0};
    float pid_val = 0.0f;
    if (sscanf(line, "pid %7s %f", pid_param, &pid_val) == 2) {
        if (strcmp(pid_param, "kp") == 0) {
            g_motor_left.pid.kp  = pid_val;
            g_motor_right.pid.kp = pid_val;
            snprintf(reply, sizeof(reply), "> Kp = %.4f\r\n", (double)pid_val);
        } else if (strcmp(pid_param, "ki") == 0) {
            g_motor_left.pid.ki  = pid_val;
            g_motor_right.pid.ki = pid_val;
            /* Сбрасываем интеграторы чтобы не было скачка */
            g_motor_left.pid.integral  = 0.0f;
            g_motor_right.pid.integral = 0.0f;
            snprintf(reply, sizeof(reply), "> Ki = %.4f  (integrators reset)\r\n", (double)pid_val);
        } else if (strcmp(pid_param, "kd") == 0) {
            g_motor_left.pid.kd  = pid_val;
            g_motor_right.pid.kd = pid_val;
            snprintf(reply, sizeof(reply), "> Kd = %.4f\r\n", (double)pid_val);
        } else {
            snprintf(reply, sizeof(reply), "> unknown pid param: %s\r\n", pid_param);
        }
        con_puts(reply);
        return;
    }

    /* ── pid show ─────────────────────────────────────────── */
    if (strcmp(line, "pid show") == 0) {
        snprintf(reply, sizeof(reply),
                 "> PID  Kp=%.4f  Ki=%.4f  Kd=%.4f\r\n",
                 (double)g_motor_left.pid.kp,
                 (double)g_motor_left.pid.ki,
                 (double)g_motor_left.pid.kd);
        con_puts(reply);
        return;
    }

    /* ── неизвестная команда ──────────────────────────────── */
    snprintf(reply, sizeof(reply), "> unknown: '%s'  (type help)\r\n", line);
    con_puts(reply);
}

/* ── Инициализация ──────────────────────────────────────────*/
void console_init(void)
{
    s_line_len   = 0U;
    s_wasd_left  = 0.0f;
    s_wasd_right = 0.0f;
    memset(s_line, 0, sizeof(s_line));
}

/* ── Задача ─────────────────────────────────────────────────*/
void task_console(void *arg)
{
    (void)arg;

    /* Небольшая пауза — ждём пока USB CDC поднимется */
    osDelay(500);

    con_puts("\r\n*** D-Grape console ready. Type 'help' for commands. ***\r\n> ");

    for (;;) {
        /* Читаем из кольцевого буфера rx_ring.
         * В DEBUG_MODE micro-ROS не запущен — rx_ring наш эксклюзивно. */

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint32_t head = rx_head;
        uint32_t tail = rx_tail;
        __set_PRIMASK(primask);

        if (head == tail) {
            /* Нет данных — уступаем CPU */
            osDelay(5);
            continue;
        }

        /* Читаем один байт */
        uint8_t b = rx_ring[tail % 512U];
        primask = __get_PRIMASK();
        __disable_irq();
        rx_tail = tail + 1U;
        __set_PRIMASK(primask);

        char c = (char)b;

        /* ── Одиночные управляющие клавиши ───────────────── */
        if (c == 'w' || c == 's' || c == 'a' || c == 'd' || c == ' ') {
            handle_key(c);
            /* Сбрасываем line-буфер — клавиша не часть команды */
            s_line_len = 0U;
            memset(s_line, 0, sizeof(s_line));
            con_puts("> ");
            continue;
        }

        /* ── Backspace ────────────────────────────────────── */
        if (c == '\b' || c == 0x7FU) {
            if (s_line_len > 0U) {
                s_line_len--;
                s_line[s_line_len] = '\0';
                con_puts("\b \b"); /* стереть символ в терминале */
            }
            continue;
        }

        /* ── Enter ────────────────────────────────────────── */
        if (c == '\r' || c == '\n') {
            con_puts("\r\n");
            if (s_line_len > 0U) {
                s_line[s_line_len] = '\0';
                handle_line(s_line);
                s_line_len = 0U;
                memset(s_line, 0, sizeof(s_line));
            }
            con_puts("> ");
            continue;
        }

        /* ── Печатаемый символ — добавляем в буфер ───────── */
        if (isprint((unsigned char)c) && s_line_len < (CONSOLE_LINE_BUF - 1U)) {
            s_line[s_line_len++] = c;
            con_echo(c); /* эхо */
        }
    }
}

#endif /* DEBUG_MODE */