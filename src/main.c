/* ============================================================
 *  main.c — D-Grape v2  (STM32F407VG Discovery)
 * ============================================================ */

#include "main.h"
#include "robot_config.h"
#include "cmsis_os2.h"
#include "freertos_app.h"

/* ── HAL handles ────────────────────────────────────────────*/
TIM_HandleTypeDef htim1;   /* PWM лыжи      PA8  */
TIM_HandleTypeDef htim2;   /* PWM моторы    PA15, PB3 */
TIM_HandleTypeDef htim3;   /* Encoder левый PA6/PA7 */
TIM_HandleTypeDef htim4;   /* Encoder правый PB6/PB7 */
TIM_HandleTypeDef htim5;   /* Encoder лыжи  PA0/PA1 */
TIM_HandleTypeDef htim6;   /* HAL timebase  */
I2C_HandleTypeDef hi2c1;   /* MPU-6050      PB8/PB9 */

/* ── Прототипы ──────────────────────────────────────────────*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_DEVICE_Init(void);

/* ── Точка входа ────────────────────────────────────────────*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();   /* 168 МГц */

    MX_GPIO_Init();         /* DIR пины PD0/PD1/PD2, LED PD12..15 */
    MX_TIM1_Init();         /* PWM лыжи 20 кГц */
    MX_TIM2_Init();         /* PWM моторы 20 кГц */
    MX_TIM3_Init();         /* Encoder левый */
    MX_TIM4_Init();         /* Encoder правый */
    MX_TIM5_Init();         /* Encoder лыжи */
    MX_TIM6_Init();         /* HAL timebase 1 кГц */
    MX_I2C1_Init();         /* MPU-6050 400 кГц */
    MX_USB_DEVICE_Init();   /* CDC для micro-ROS */

    freertos_app_init();    /* Задачи + аппаратура */
    osKernelStart();

    /* Сюда не доходим */
    for (;;) {}
}

/* ── Тактирование 168 МГц ───────────────────────────────────
 *  HSE = 8 МГц (кварц на Discovery)
 *  PLL: M=8, N=336, P=2, Q=7
 *  SYSCLK = 168 МГц, APB1 = 42 МГц (×2 → TIM = 84 МГц)
 *           APB2 = 84 МГц (×2 → TIM = 168 МГц)
 *  USB = 48 МГц (168/Q=7 * 2 = 48)
 * ────────────────────────────────────────────────────────── */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM            = 8;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/* ── GPIO: DIR пины + LED ───────────────────────────────────*/
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = MOTOR_LEFT_DIR_PIN | MOTOR_RIGHT_DIR_PIN | MOTOR_SKIS_DIR_PIN;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* Встроенные LED (PD12..15) для отладки */
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);
}

/* ── TIM1: PWM лыжи, PA8, AF1, 20 кГц ──────────────────────
 *  APB2 TIM clock = 168 МГц
 *  PSC=0, ARR=8399 → f = 168 000 000 / (0+1) / (8399+1) = 20 000 Гц
 * ────────────────────────────────────────────────────────── */
static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 8399;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
}

/* ── TIM2: PWM моторы, PA15/PB3, AF1, 20 кГц ───────────────
 *  APB1 TIM clock = 84 МГц
 *  PSC=0, ARR=4199 → f = 84 000 000 / (0+1) / (4199+1) = 20 000 Гц
 * ────────────────────────────────────────────────────────── */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    gpio.Pin       = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin       = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim2.Instance           = TIM2;
    htim2.Init.Prescaler     = 0;
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 4199;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
}

/* ── TIM3: Encoder левый, PA6/PA7, AF2 ─────────────────────*/
static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim3.Instance           = TIM3;
    htim3.Init.Prescaler     = 0;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = ENCODER_TIMER_PERIOD;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim3, &(TIM_Encoder_InitTypeDef){
        .EncoderMode  = TIM_ENCODERMODE_TI12,
        .IC1Polarity  = TIM_ICPOLARITY_RISING,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1,
        .IC1Filter    = 4,
        .IC2Polarity  = TIM_ICPOLARITY_RISING,
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1,
        .IC2Filter    = 4,
    });
}

/* ── TIM4: Encoder правый, PB6/PB7, AF2 ────────────────────*/
static void MX_TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim4.Instance           = TIM4;
    htim4.Init.Prescaler     = 0;
    htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim4.Init.Period        = ENCODER_TIMER_PERIOD;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim4, &(TIM_Encoder_InitTypeDef){
        .EncoderMode  = TIM_ENCODERMODE_TI12,
        .IC1Polarity  = TIM_ICPOLARITY_RISING,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1,
        .IC1Filter    = 4,
        .IC2Polarity  = TIM_ICPOLARITY_RISING,
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1,
        .IC2Filter    = 4,
    });
}

/* ── TIM5: Encoder лыжи, PA0/PA1, AF2 ──────────────────────*/
static void MX_TIM5_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim5.Instance           = TIM5;
    htim5.Init.Prescaler     = 0;
    htim5.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim5.Init.Period        = 0xFFFFFFFFU; /* TIM5 — 32-бит! */
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim5, &(TIM_Encoder_InitTypeDef){
        .EncoderMode  = TIM_ENCODERMODE_TI12,
        .IC1Polarity  = TIM_ICPOLARITY_RISING,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1,
        .IC1Filter    = 4,
        .IC2Polarity  = TIM_ICPOLARITY_RISING,
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1,
        .IC2Filter    = 4,
    });
}

/* ── TIM6: HAL timebase 1 кГц ──────────────────────────────*/
static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance         = TIM6;
    htim6.Init.Prescaler   = (uint32_t)((SystemCoreClock / 2) / 1000000U) - 1;
    htim6.Init.Period      = (1000000U / 1000U) - 1; /* 1 кГц */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&htim6);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ── I2C1: MPU-6050, PB8/PB9, 400 кГц ─────────────────────
 *  Fast mode: CCR = 42 МГц / (400 кГц * 3) = 35
 *  Trise = 42 МГц * 300нс + 1 = 13
 * ────────────────────────────────────────────────────────── */
static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;  /* SCL / SDA */
    gpio.Mode      = GPIO_MODE_AF_OD;           /* Open-drain для I2C */
    gpio.Pull      = GPIO_NOPULL;               /* внешние pull-up 4.7кОм */
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* ── USB CDC (micro-ROS transport) ──────────────────────────*/
static void MX_USB_DEVICE_Init(void)
{
    /* Реализация зависит от micro_ros_stm32cubemx_utils.
     * Файлы usb_device.c / usb_transport.c берутся из репозитория.
     * Здесь — placeholder. */
    extern void MX_USB_DEVICE_Init_impl(void);
    MX_USB_DEVICE_Init_impl();
}
/* ── USB stub — подключается с micro-ROS ──────────────────── */
void MX_USB_DEVICE_Init_impl(void) {}