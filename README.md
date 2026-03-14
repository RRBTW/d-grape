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
| Энкодеры | Квадратурные, 1024 PPR |
| IMU | MPU-6050, I2C1, 500 Гц |
| Связь | USB CDC, PA11/PA12, OTG_FS |
| Отладка | ST-Link встроенный |

---

## Сборка и прошивка

```bash
# Сборка
pio run

# Прошивка (ST-Link)
pio run --target upload
# или Ctrl+Alt+U в VS Code
```

Требования: PlatformIO, WSL Ubuntu (для пересборки libmicroros.a — не обязательно, .a уже в репозитории).

---

## Запуск micro-ROS агента на PC

```bash
# Docker (рекомендуется)
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200

# Или нативно (ROS 2 Humble)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### Проверка топиков

```bash
ros2 topic list

# Данные скорости и угловой скорости
ros2 topic echo /d_grape/velocity

# Управление гусеницами (x=левая м/с, y=правая м/с)
ros2 topic pub /d_grape/wheel_cmd geometry_msgs/msg/Vector3 "{x: 0.3, y: 0.3}"
```

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
├── platformio.ini              ← конфигурация сборки
├── extra_scripts.py            ← FPU патч + линковка libmicroros.a
├── include/
│   ├── main.h                  ← extern HAL handles
│   ├── robot_config.h          ← все физические константы
│   ├── freertos_app.h          ← публичный API приложения
│   ├── FreeRTOSConfig.h        ← конфигурация FreeRTOS
│   ├── stm32f4xx_it.h          ← прерывания
│   └── usbd_conf.h             ← конфигурация USB Device стека
├── src/
│   ├── main.c                  ← точка входа, инициализация HAL
│   ├── freertos_app.c          ← ядро приложения, все FreeRTOS задачи
│   ├── stm32f4xx_it.c          ← обработчики прерываний
│   ├── microros_transport.c    ← USB CDC транспорт для micro-ROS
│   └── microros_time.c         ← clock_gettime() через HAL_GetTick()
└── lib/
    ├── motors/                 ← драйвер моторов (PWM + DIR)
    ├── encoders/               ← квадратурные энкодеры
    ├── pid/                    ← ПИД регулятор скорости
    ├── imu/                    ← драйвер MPU-6050
    ├── kalman/                 ← фильтр Калмана (IMU + энкодер)
    ├── freertos/               ← FreeRTOS + CMSIS_RTOS_V2
    ├── usb_cdc/                ← полный USB CDC стек (без CubeMX)
    │   ├── usbd_conf.c/h       ← HAL PCD callbacks + LL interface
    │   ├── usbd_desc.c/h       ← USB дескрипторы (VID/PID/Serial)
    │   ├── usbd_cdc_if.c/h     ← CDC интерфейс (Rx/Tx буферы)
    │   └── usb_device.c/h      ← MX_USB_DEVICE_Init()
    └── micro_ros_platformio/
        ├── libmicroros/
        │   ├── libmicroros.a   ← скомпилированная micro-ROS (~10 МБ)
        │   └── include/        ← заголовки ROS 2
        └── microros_utils/     ← скрипты пересборки библиотеки
```

---

## Архитектура

### FreeRTOS задачи

| Задача | Частота | Приоритет | Назначение |
|---|---|---|---|
| `task_imu` | 500 Гц | AboveNormal | Читает IMU → `g_imu_raw` [mutex] |
| `task_robot` | 100 Гц | Normal | Энкодеры → Калман → ПИД → моторы |
| `task_microros` | 40 Гц | Normal | micro-ROS spin + publish |
| `task_watchdog` | событийная | High | Аварийный стоп + сброс при зависании |

### Поток данных `task_robot` (100 Гц)

```
IMU 500Гц ──► g_imu_raw [mutex]
                    │
Encoders ──► speed_mps ──► kf_vel_update() ──► velocity_fused
                       └── kf_yaw_update() ──► omega_fused
                                                     │
                                               g_kf_out [mutex]
                                                     │
                         g_cmd_mps (← ROS) ──► motor_velocity_update()
                                                     │ PID
                                               motor_set_duty()
                                                     │
                                              TIM CCR ──► Motors
```

### USB CDC транспортный уровень

```
ROS 2 Agent (PC)
      │ USB
      ▼
OTG_FS_IRQHandler
      │
CDC_Receive_FS()
      │
microros_usb_recv_cb() ──► rx_ring[512]
                                  │
                    task_microros ◄── usb_cdc_transport_read()
                    task_microros ──► usb_cdc_transport_write() ──► CDC_Transmit_FS()
```

---

## Описание файлов

### `src/main.c`
Точка входа. Инициализирует всю периферию через HAL и запускает FreeRTOS:
- `SystemClock_Config()` — 168 МГц (HSE 8 МГц + PLL)
- `MX_TIM1_Init()` — PWM лыжи, PA8, 20 кГц
- `MX_TIM2_Init()` — PWM моторы, PA15+PB3, 20 кГц
- `MX_TIM3/4/5_Init()` — энкодеры (encoder interface mode)
- `MX_TIM6_Init()` — HAL timebase 1 кГц (не SysTick)
- `MX_I2C1_Init()` — IMU MPU-6050
- `MX_USB_DEVICE_Init()` — USB CDC стек
- `freertos_app_init()` + `osKernelStart()`

### `src/freertos_app.c`
Ядро приложения. Содержит все задачи, логику управления, инициализацию micro-ROS.

`ros_init()` при старте: устанавливает USB CDC транспорт (`rmw_uros_set_custom_transport`), пингует агента, создаёт ноду и топики. При обрыве связи — перезапускается через `osDelay(1000)`.

### `src/microros_transport.c`
Кастомный транспорт micro-ROS через USB CDC. Кольцевой буфер `rx_ring[512]` наполняется в `microros_usb_recv_cb()` из прерывания USB. `transport_write()` — блокирующий с таймаутом 20 мс.

### `src/microros_time.c`
POSIX `clock_gettime()` для голого железа. micro-ROS требует эту функцию для временны́х меток. Реализована через `HAL_GetTick()`, точность ±1 мс.

### `include/robot_config.h`
Все физические константы в одном месте. **Именно здесь** нужно менять параметры при настройке:

```c
#define ROBOT_WHEEL_RADIUS  0.0877f   // измерить штангенциркулем
#define ROBOT_TRACK_WIDTH   0.526f    // расстояние между серединами гусениц
#define PID_KP  1.5f                  // настроить на реальном железе
#define PID_KI  0.1f
#define PID_KD  0.05f
#define CMD_TIMEOUT_MS  500U          // стоп при обрыве связи
#define IMU_SMPLRT_DIV  1U            // 500 Гц при DLPF=3
```

### `lib/usb_cdc/`
Полный USB CDC стек без CubeMX. Написан вручную совместимый со старым API STM32CubeF4:
- `usbd_conf.c` — настройка GPIO PA11/PA12, PCD callbacks, FIFO (Rx=128Б, Tx=64+128Б)
- `usbd_desc.c` — VID=0x0483, PID=0x5740, серийный номер из UID процессора
- `usbd_cdc_if.c` — буферы RX/TX по 512 байт, callback при получении данных
- `usb_device.c` — `MX_USB_DEVICE_Init()`, вызывается из `main.c`

### `lib/pid/`
ПИД с D-term на измерении (не на ошибке) — устраняет derivative kick при скачке уставки. Anti-windup через ограничение интегратора. Выход нормирован в `[-1.0, +1.0]`.

### `lib/kalman/`
2D фильтр Калмана. Два независимых экземпляра:
- **kf_vel**: состояние `[velocity, accel_bias]`, измерения: акселерометр + энкодер
- **kf_yaw**: состояние `[omega, gyro_bias]`, измерения: гироскоп + энкодер

### `lib/imu/`
Драйвер MPU-6050 по I2C. 14-байтное burst-чтение accel+temp+gyro. Конвертация в SI: акселерометр → м/с², гироскоп → рад/с. DLPF=3 даёт LPF 42 Гц.

### `lib/encoders/`
Квадратурные энкодеры через TIM encoder interface. Обработка переполнения ±32767. Формула скорости: `speed = (delta / PPR) × 2π × R / dt`.

### `lib/motors/`
Драйвер 3 моторов. Знак duty cycle определяет DIR пин, модуль — регистр CCR таймера. Встроенный ПИД контур скорости через `motor_velocity_update()`.

---

## Использование памяти

| | Значение |
|---|---|
| RAM | 51.2% (67 КБ / 131 КБ) |
| Flash | 10.6% (111 КБ / 1 МБ) |
| FreeRTOS Heap | 30 КБ |
| USB буферы | 2 × 512 Б |
| micro-ROS RX ring | 512 Б |

---

## Пересборка libmicroros.a

> Нужно только если хочешь добавить новые ROS 2 пакеты или изменить конфигурацию.
> В репозитории уже есть готовая `.a` для STM32F407.

Требуется: WSL Ubuntu, `colcon`, `arm-none-eabi-gcc`.

```bash
# В WSL
cd /home/roma/micro_ros_build
colcon build --merge-install \
  --packages-ignore lttngpy rcl_logging_spdlog test_tracetools rclc_examples \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=<path>/platformio_toolchain.cmake \
    -DMICROROS_TRANSPORT=custom
```

После сборки скопировать `.a` и `include/` в `lib/micro_ros_platformio/libmicroros/`.
