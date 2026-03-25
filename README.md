# D-Grape

Прошивка гусеничного робота на **STM32F407VG Discovery**.
Написана на C, собирается в **PlatformIO** (`stm32cube` framework).
Связь с ROS 2 через **micro-ROS** по **USB CDC** (виртуальный COM-порт).

---

## Аппаратура

| Компонент | Описание |
|---|---|
| МК | STM32F407VGT6, 168 МГц, 1 МБ Flash, 192 КБ RAM |
| Плата | STM32F407VG Discovery |
| Драйверы моторов | BTS7960B × 2 (dual PWM: RPWM/LPWM, без DIR) |
| Моторы | 2 гусеничных + 1 лыжи |
| Частота ШИМ | 20 кГц |
| Энкодеры | **не подключены** (open-loop режим) |
| IMU | MPU-6050, I2C1, 500 Гц |
| Связь | USB CDC, PA11/PA12, OTG_FS |
| Отладка | ST-Link встроенный |

---

## Быстрый старт

### Нормальный режим (micro-ROS)

В `include/robot_config.h` строка `#define DEBUG_MODE` должна быть **закомментирована**.

```bash
pio run                      # сборка
pio run --target upload      # прошивка
```

Запустить micro-ROS агента на PC:

```bash
# Docker (рекомендуется)
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200

# Или нативно (ROS 2 Humble)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Проверка топиков:

```bash
ros2 topic list
ros2 topic echo /d_grape/velocity
ros2 topic pub /d_grape/wheel_cmd geometry_msgs/msg/Vector3 "{x: 0.3, y: -0.3}"
```

> ⚠️ Правый мотор стоит **физически инвертированно** — для движения вперёд посылай `x = +v, y = -v`.

### Отладочный режим (без ROS)

Раскомментируй в `include/robot_config.h`:

```c
#define DEBUG_MODE
```

Пересобери и прошей. Открой любой Serial Monitor на `COMx` / `/dev/ttyACM0`.

Что получишь:
- Телеметрия 10 Гц — IMU, фильтр Калмана, PID, USB статус
- Интерактивная консоль управления

**Клавиши (без Enter):**

| Клавиша | Действие |
|---|---|
| `w` | вперёд (+0.1 м/с к обеим гусеницам) |
| `s` | назад (−0.1 м/с) |
| `a` | поворот влево |
| `d` | поворот вправо |
| пробел | стоп |

**Текстовые команды (+ Enter):**

| Команда | Действие |
|---|---|
| `left 0.3 right -0.3` | прямая уставка скоростей [м/с] |
| `stop` | аварийный стоп + сброс PID |
| `reset` | сбросить уставки и WASD в 0 |
| `pid kp 0.5` | изменить Kp на лету |
| `pid ki 0.1` | изменить Ki (сбрасывает интегратор) |
| `pid kd 0.05` | изменить Kd |
| `pid show` | показать текущие коэффициенты |
| `help` | список всех команд |

> В отладочном режиме `task_microros` не запускается — USB занят консолью и телеметрией.

---

## ROS 2 топики

| Топик | Тип | Направление |
|---|---|---|
| `/d_grape/wheel_cmd` | `geometry_msgs/Vector3` {x=left, y=right м/с} | Subscribe |
| `/d_grape/imu/filtered` | `sensor_msgs/Imu` | Publish |
| `/d_grape/velocity` | `geometry_msgs/Vector3` {x=v м/с, z=ω рад/с} | Publish |
| `/d_grape/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Publish |

---

## Разводка пинов

| Функция | Пин | Периферия |
|---|---|---|
| Левый RPWM (вперёд) | PA15 | TIM2 CH1 AF1 |
| Левый LPWM (назад) | PA7 | TIM3 CH2 AF2 |
| Правый RPWM (вперёд) | PB3 | TIM2 CH2 AF1 |
| Правый LPWM (назад) | PB11 | TIM2 CH4 AF1 |
| PWM лыжи | PA8 | TIM1 CH1 AF1 |
| DIR лыжи | PD2 | GPIO OUT |
| Encoder левый A/B | PA6/PA7 | TIM3 AF2 ⚠️ занят LPWM |
| Encoder правый A/B | PB6/PB7 | TIM4 AF2 |
| Encoder лыжи A/B | PA0/PA1 | TIM5 AF2 |
| USB D−/D+ | PA11/PA12 | OTG_FS AF10 |
| IMU SCL | PB8 | I2C1 AF4 |
| IMU SDA | PB9 | I2C1 AF4 |
| LED зелёный (IMU) | PD12 | GPIO OUT |
| LED оранжевый (моторы) | PD13 | GPIO OUT |
| LED красный (heartbeat) | PD14 | GPIO OUT |
| LED синий (связь) | PD15 | GPIO OUT |
| LED красный (авария) | PE1 | GPIO OUT |

> ⚠️ При подключении энкодеров: перенести левый LPWM с PA7 (TIM3_CH2) на PB10 (TIM2_CH3), чтобы освободить TIM3 под левый энкодер.

---

## Структура проекта

```
d-grape/
├── platformio.ini
├── extra_scripts.py            ← FPU патч + линковка libmicroros.a
├── include/
│   ├── robot_config.h          ← ВСЕ константы (менять только здесь)
│   ├── freertos_app.h          ← публичный API + extern глобальных объектов
│   ├── main.h                  ← extern HAL handles
│   ├── FreeRTOSConfig.h
│   └── stm32f4xx_it.h
├── src/
│   ├── main.c                  ← инициализация HAL, TIM, I2C, запуск FreeRTOS
│   ├── freertos_app.c          ← все задачи, логика управления
│   ├── microros_transport.c    ← rx_ring[512], USB CDC транспорт micro-ROS
│   └── stm32f4xx_it.c
└── lib/
    ├── motors/                 ← BTS7960B dual PWM (RPWM/LPWM) + встроенный PID
    ├── encoders/               ← TIM encoder mode (не используется, готово к подключению)
    ├── pid/                    ← D на измерении, anti-windup
    ├── imu/                    ← MPU-6050, I2C burst 14 байт, 500 Гц
    ├── kalman/                 ← 2D KF: predict-only (без энкодеров) / fusion (с энкодерами)
    ├── debug/                  ← только при DEBUG_MODE
    │   ├── debug.c/h           ← телеметрия 10 Гц по USB CDC
    │   └── console.c/h         ← интерактивная консоль WASD
    ├── freertos/               ← FreeRTOS + CMSIS_RTOS_V2
    ├── usb_cdc/                ← USB CDC стек (cdc_rx_hook weak callback)
    └── micro_ros_platformio/
        └── libmicroros/        ← libmicroros.a + include/ (ROS 2 Humble)
```

---

## Архитектура

### FreeRTOS задачи — нормальный режим

| Задача | Частота | Приоритет | Назначение |
|---|---|---|---|
| `task_imu` | 500 Гц | AboveNormal | IMU → `g_imu_raw` [mutex] |
| `task_robot` | 100 Гц | Normal | IMU → Калман → ПИД → моторы |
| `task_microros` | 40 Гц | Normal | micro-ROS spin + publish + reconnect |
| `task_watchdog` | событийная | High | Аварийный стоп + SystemReset при зависании (200 мс таймаут) |

### FreeRTOS задачи — отладочный режим

| Задача | Частота | Приоритет | Назначение |
|---|---|---|---|
| `task_imu` | 500 Гц | AboveNormal | то же |
| `task_robot` | 100 Гц | Normal | то же |
| `task_debug` | 10 Гц | Normal | телеметрия → USB CDC |
| `task_console` | событийная | Normal | парсинг WASD + команд из USB CDC |
| `task_watchdog` | событийная | High | то же |

### Поток данных

```
IMU 500 Гц ──► g_imu_raw [mutex]
                     │  ax, gz
                     ▼
              kf_vel_predict_only()  ──► velocity_fused [м/с]
              kf_yaw_predict_only()  ──► omega_fused [рад/с]
                     │
               g_kf_out [mutex]
                     │
g_cmd_mps ◄── ROS /wheel_cmd        ──► motor_velocity_update()
           ◄── console (WASD)              │ open-loop duty
                                     TIM CCR ──► BTS7960B ──► Motors
```

### USB CDC

```
ROS 2 Agent / Serial Monitor
    │ USB
    ▼
OTG_FS_IRQ ──► CDC_Receive_FS() ──► cdc_rx_hook() ──► rx_ring[512]
                                                              │
                                        task_microros / task_console ◄──┘
                                        task_microros / task_debug ──► CDC_Transmit_FS()
```

---

## Настройка параметров

Все параметры в `include/robot_config.h`:

```c
/* Физика */
#define ROBOT_WHEEL_RADIUS   0.0877f  // радиус звёздочки [м]
#define ROBOT_TRACK_WIDTH    0.526f   // полная колея [м]

/* PID (сейчас open-loop, KI/KD = 0) */
#define PID_KP               0.3f
#define PID_KI               0.0f
#define PID_KD               0.0f

/* Safety */
#define CMD_TIMEOUT_MS       500U     // стоп при обрыве связи [мс]

/* IMU */
#define IMU_SMPLRT_DIV       1U       // 500 Гц при DLPF=3

/* Отладочный режим */
/* #define DEBUG_MODE */
```

---

## Использование памяти

| Режим | RAM | Flash |
|---|---|---|
| DEBUG_MODE | 53% (69 КБ / 131 КБ) | 4.1% (43 КБ / 1 МБ) |
| micro-ROS | 74.5% (97 КБ / 131 КБ) | 10.5% (110 КБ / 1 МБ) |

---

## Подключение энкодеров (в будущем)

1. Перенести левый LPWM с **PA7** (TIM3_CH2) на **PB10** (TIM2_CH3, AF1)
2. В `include/robot_config.h` обновить:
   ```c
   #define MOTOR_LEFT_CH_REV    TIM_CHANNEL_3   /* PB10, TIM2 */
   #define MOTOR_LEFT_CCR_REV   (TIM2->CCR3)
   ```
3. В `src/main.c` переключить `MX_TIM3_Init()` обратно в encoder mode (PA6/PA7)
4. В `src/freertos_app.c` раскомментировать `encoder_init` и переключить KF на `kf_vel_update` / `kf_yaw_update`

---

## Пересборка libmicroros.a

> Нужно только при добавлении новых ROS 2 пакетов. Готовая `.a` уже в репозитории.

Требуется: WSL Ubuntu, `colcon`, `arm-none-eabi-gcc`.

```bash
cd /home/user/micro_ros_build
colcon build --merge-install \
  --packages-ignore lttngpy rcl_logging_spdlog test_tracetools rclc_examples \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=<path>/platformio_toolchain.cmake \
    -DMICROROS_TRANSPORT=custom
```

После сборки скопировать `.a` и `include/` в `lib/micro_ros_platformio/libmicroros/`.
