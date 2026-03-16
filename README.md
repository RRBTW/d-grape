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
| Моторы | 2 гусеничных + 1 лыжи, PWM 20 кГц |
| Энкодеры | Квадратурные 1024 PPR, режим TI12 (4096 отсчётов/об) |
| IMU | MPU-6050, I2C1, 500 Гц |
| Связь | USB CDC, PA11/PA12, OTG_FS |
| Отладка | ST-Link встроенный |

---

## Быстрый старт

### Нормальный режим (micro-ROS)

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
ros2 topic pub /d_grape/wheel_cmd geometry_msgs/msg/Vector3 "{x: 0.3, y: 0.3}"
```

### Отладочный режим (без ROS)

Раскомментируй в `include/robot_config.h`:

```c
#define DEBUG_MODE
```

Пересобери и прошей. Открой любой Serial Monitor на `COMx` / `/dev/ttyACM0`, **115200 бод**.

Что получишь:
- Телеметрия 10 Гц — IMU, энкодеры, фильтр Калмана, PID
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
| `left 0.3 right 0.3` | прямая уставка скоростей [м/с] |
| `stop` | аварийный стоп + сброс PID |
| `reset` | обнулить энкодеры |
| `pid kp 2.0` | изменить Kp на лету |
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
| PWM левый мотор | PA15 | TIM2 CH1 AF1 |
| PWM правый мотор | PB3 | TIM2 CH2 AF1 |
| PWM лыжи | PA8 | TIM1 CH1 AF1 |
| DIR левый | PD0 | GPIO OUT |
| DIR правый | PD1 | GPIO OUT |
| DIR лыжи | PD2 | GPIO OUT |
| Encoder левый A/B | PA6/PA7 | TIM3 AF2 |
| Encoder правый A/B | PB6/PB7 | TIM4 AF2 |
| Encoder лыжи A/B | PA0/PA1 | TIM5 AF2 |
| USB D−/D+ | PA11/PA12 | OTG_FS AF10 |
| USB VBUS sense | PA9 | GPIO IN |
| HAL Timebase | — | TIM6, 1 кГц |
| IMU SCL | PB8 | I2C1 AF4 |
| IMU SDA | PB9 | I2C1 AF4 |

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
│   ├── stm32f4xx_it.h
│   └── usbd_conf.h
├── src/
│   ├── main.c                  ← инициализация HAL, запуск FreeRTOS
│   ├── freertos_app.c          ← все задачи, логика управления
│   ├── microros_transport.c    ← rx_ring, USB CDC транспорт
│   ├── microros_time.c         ← clock_gettime() для micro-ROS
│   └── stm32f4xx_it.c
└── lib/
    ├── motors/                 ← PWM + DIR + встроенный PID
    ├── encoders/               ← TIM encoder mode, обработка переполнения
    ├── pid/                    ← D на измерении, anti-windup
    ├── imu/                    ← MPU-6050, I2C burst 14 байт
    ├── kalman/                 ← 2D KF: velocity + yaw fusion
    ├── debug/                  ← только при DEBUG_MODE
    │   ├── debug.h / debug.c   ← телеметрия 10 Гц
    │   └── console.h / console.c ← интерактивная консоль
    ├── freertos/               ← FreeRTOS + CMSIS_RTOS_V2
    ├── usb_cdc/                ← USB CDC стек без CubeMX
    └── micro_ros_platformio/
        └── libmicroros/        ← libmicroros.a + include/
```

---

## Архитектура

### FreeRTOS задачи — нормальный режим

| Задача | Частота | Приоритет | Назначение |
|---|---|---|---|
| `task_imu` | 500 Гц | AboveNormal | IMU → `g_imu_raw` [mutex] |
| `task_robot` | 100 Гц | Normal | Энкодеры → Калман → ПИД → моторы |
| `task_microros` | 40 Гц | Normal | micro-ROS spin + publish |
| `task_watchdog` | событийная | High | Аварийный стоп + сброс при зависании |

### FreeRTOS задачи — отладочный режим

| Задача | Частота | Приоритет | Назначение |
|---|---|---|---|
| `task_imu` | 500 Гц | AboveNormal | то же |
| `task_robot` | 100 Гц | Normal | то же |
| `task_debug` | 10 Гц | Normal | телеметрия → USB CDC |
| `task_console` | событийная | Normal | парсинг команд из USB CDC |
| `task_watchdog` | событийная | High | то же |

### Поток данных

```
IMU 500Гц ──► g_imu_raw [mutex]
                    │
Encoders ──► speed_mps ──► kf_vel_update() ──► velocity_fused
                       └── kf_yaw_update() ──► omega_fused
                                                     │
                                               g_kf_out [mutex]
                                                     │
              g_cmd_mps ◄── ROS / console ──► motor_velocity_update()
                                                     │ PID
                                               motor_set_duty()
                                                     │
                                             TIM CCR ──► Motors
```

### USB CDC — нормальный режим

```
ROS 2 Agent ──USB──► OTG_FS_IRQ ──► CDC_Receive_FS()
                                          │
                              microros_usb_recv_cb() ──► rx_ring[512]
                                                               │
                                             task_microros ◄──┘
                                             task_microros ──► CDC_Transmit_FS()
```

### USB CDC — отладочный режим

```
Serial Monitor ──USB──► OTG_FS_IRQ ──► CDC_Receive_FS()
                                             │
                             microros_usb_recv_cb() ──► rx_ring[512]
                                                              │
                                            task_console ◄───┘  (парсинг)
                                            task_debug ──► CDC_Transmit_FS()  (телеметрия)
                                            task_console ──► CDC_Transmit_FS() (ответы)
```

---

## Настройка параметров

Все параметры в `include/robot_config.h`:

```c
/* Физика */
#define ROBOT_WHEEL_RADIUS   0.0877f  // измерить штангенциркулем [м]
#define ROBOT_TRACK_WIDTH    0.526f   // между серединами гусениц [м]

/* Энкодеры */
#define ENCODER_PPR          4096U    // TI12 режим: 1024 PPR × 4 = 4096 CPR

/* PID */
#define PID_KP               1.5f
#define PID_KI               0.1f
#define PID_KD               0.05f

/* Safety */
#define CMD_TIMEOUT_MS       500U     // стоп при обрыве связи [мс]

/* IMU */
#define IMU_SMPLRT_DIV       1U       // 500 Гц при DLPF=3
#define IMU_I2C_TIMEOUT_MS   1U       // < периода задачи 2 мс

/* Отладочный режим */
/* #define DEBUG_MODE */
```

---

## Использование памяти

| | Значение |
|---|---|
| RAM | 51.2% (67 КБ / 131 КБ) |
| Flash | 10.6% (111 КБ / 1 МБ) |
| FreeRTOS Heap | 30 КБ |
| USB буферы | 2 × 512 Б |
| RX ring | 512 Б |

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