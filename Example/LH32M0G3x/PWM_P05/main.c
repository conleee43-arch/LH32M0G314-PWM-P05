/******************************************************************************
 * 版权所有：苏州领慧立芯科技有限公司
 * Copyright (c) 2020-2025 Suzhou Legendsemi Technology Co., Ltd.
 ******************************************************************************
 * All rights reserved. Distributed under MIT license.
 * The file is encoded in UTF-8 without signature.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lh32m0xx_lhl.h"

/* Private define ------------------------------------------------------------*/
#define TEMPLATE_LED_PORT             pGPIO0
#define TEMPLATE_LED_PIN              GPIO_PIN_0
#define TEMPLATE_HEARTBEAT_PERIOD_US  1000000UL
#define PWM_P05_DEFAULT_FREQ_HZ       1000U
#define PWM_P05_DEFAULT_DUTY_PCT      20U

/* Private declaration -------------------------------------------------------*/
void SystemClock_Config(void);
void GPIO_Config(void);
void P05_PWM_Init(uint32_t freq_hz, uint8_t duty_percent);
void P05_PWM_SetDuty(uint8_t duty_percent);
static uint16_t P05_PWM_CalcPulse(uint16_t period, uint8_t duty_percent);

/* Functions -----------------------------------------------------------------*/
int main(void)
{
    /* 上电后IWDG默认开启，若无处理约20秒后复位 */
    LHL_IWDG_Stop();

    /* 1. 配置系统时钟到4MHz，给GPIO/TIM1提供稳定时钟 */
    SystemClock_Config();

    /* 2. 配置LED输出引脚 */
    GPIO_Config();

    /* 3. 初始化P0.5(TIM1_CH1)输出指定频率和占空比的持续PWM波形 */
    P05_PWM_Init(PWM_P05_DEFAULT_FREQ_HZ, PWM_P05_DEFAULT_DUTY_PCT);
	P05_PWM_SetDuty(50);
    while(1)
    {
        /* 每1秒翻转LED */
        LHL_Delay_uS(TEMPLATE_HEARTBEAT_PERIOD_US);
        LHL_GPIO_TogglePin(TEMPLATE_LED_PORT, TEMPLATE_LED_PIN);
    }
}

/**
  * @brief  根据 ARR（Period）和占空比百分比（0~100）计算 CCR1（Pulse）比较值
  * @param  period:       TIM1 自动重装载值 ARR（实际计数周期为 period + 1）
  * @param  duty_percent: 目标占空比百分比（0 ~ 100）
  * @retval 写入 CCR1 的 16 位比较值
  */
static uint16_t P05_PWM_CalcPulse(uint16_t period, uint8_t duty_percent)
{
    uint32_t pulse;

    /* 限幅保护：占空比最大 100% */
    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    /* 向上计数模式下，1个PWM周期共 (ARR + 1) 个计数脉冲；+50 实现四舍五入 */
    pulse = (((uint32_t)period + 1U) * (uint32_t)duty_percent + 50U) / 100U;

    /* 防止 CCR1 超过 16 位寄存器上限 0xFFFF */
    if (pulse > 0xFFFFU)
    {
        pulse = 0xFFFFU;
    }

    return (uint16_t)pulse;
}

/**
  * @brief  初始化 P0.5 引脚为 TIM1_CH1 并输出指定频率和占空比的持续 PWM 波形
  * @param  freq_hz:      目标 PWM 频率（单位：Hz，需 > 0）
  * @param  duty_percent: 占空比百分比（0 ~ 100）
  * @retval None
  * @note   当 APB2 分频为 1 分频（RCC_HCLK_DIV1）时，TIM1 时钟频率等于 SystemCoreClock
  */
void P05_PWM_Init(uint32_t freq_hz, uint8_t duty_percent)
{
    GPIO_InitTypeDef    GPIO_InitStructure    = {0};
    TIM_InitTypeDef     TIM_InitStructure     = {0};
    TIM_PWMInitTypeDef  TIM_PWMInitStructure  = {0};

    uint32_t total_ticks;
    uint32_t psc_div;
    uint32_t arr_ticks;
    uint16_t prescaler;
    uint16_t period;
    uint16_t pulse;

    /* 参数合法性检查，避免除零 */
    if ((freq_hz == 0U) || (SystemCoreClock == 0U))
    {
        return;
    }

    /* 1. 计算单个 PWM 周期需要的定时器输入时钟总脉冲数（四舍五入） */
    total_ticks = (SystemCoreClock + (freq_hz / 2U)) / freq_hz;
    if (total_ticks < 2U)
    {
        total_ticks = 2U;   /* 至少需要 2 个计数周期（ARR >= 1，ARR=0 时计数器不工作） */
    }

    /*
     * 2. 自适应求解最小分频倍率 psc_div，使每周期计数点数 arr_ticks <= 65535（即 ARR <= 65534）：
     *    在 16 位不溢出前提下最大化 ARR 以获得最高占空比分辨率，
     *    并保证 ARR + 1 <= 65535，使 100% 占空比时 CCR1 = ARR + 1 不超出 16 位范围。
     */
    psc_div = ((total_ticks - 1U) / 65535U) + 1U;
    if (psc_div > 65536U)
    {
        psc_div = 65536U;
    }

    /* 3. 计算重装载计数点数 arr_ticks（四舍五入并限制在 [2, 65535]） */
    arr_ticks = (total_ticks + (psc_div / 2U)) / psc_div;
    if (arr_ticks < 2U)
    {
        arr_ticks = 2U;
    }
    else if (arr_ticks > 65535U)
    {
        arr_ticks = 65535U;
    }

    /* 硬件寄存器值为实际倍率/点数减 1 */
    prescaler = (uint16_t)(psc_div - 1U);
    period    = (uint16_t)(arr_ticks - 1U);
    pulse     = P05_PWM_CalcPulse(period, duty_percent);

    /* 4. 配置 P0.5 引脚为复用推挽输出，映射为 TIM1_CH1（LHL_GPIO_Init 内部已使能 GPIO 时钟） */
    GPIO_InitStructure.Pin            = GPIO_PIN_5;
    GPIO_InitStructure.Mode           = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current        = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull           = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate      = GPIO0_5_AF_TIM1_CH1;
    LHL_GPIO_Init(pGPIO0, &GPIO_InitStructure);

    /* 5. 复位并初始化 TIM1 时基，开启 ARR 预装载（LHL_TIM_Init 内部已使能 TIM1 时钟） */
    LHL_TIM_DeInit(pTIM1);

    TIM_InitStructure.Prescaler         = prescaler;
    TIM_InitStructure.Period            = period;
    TIM_InitStructure.CounterMode       = TIM_COUNTERMODE_UP;
    TIM_InitStructure.AutoReloadPreload = ENABLE;
    TIM_InitStructure.ClockDivision     = TIM_CKD_DIV1;
    LHL_TIM_Init(pTIM1, &TIM_InitStructure);

    /* 6. 初始化 TIM1 通道 1 为 PWM 模式 1、高电平有效并开启 CCR1 预装载 */
    TIM_PWMInitStructure.PWMMode     = TIM_PWMMODE_PWM1;
    TIM_PWMInitStructure.PWMPolarity = TIM_PWMPOLARITY_ACTIVE_HIGH;
    TIM_PWMInitStructure.PWMFastMode = DISABLE;
    TIM_PWMInitStructure.PWMPreload  = ENABLE;
    TIM_PWMInitStructure.Pulse       = pulse;
    /* LHL_TIM_PWM_Init 内部会自动开启 CC1E 通道输出并产生 UG 事件装载寄存器 */
    LHL_TIM_PWM_Init(pTIM1, &TIM_PWMInitStructure, TIM_CHANNEL_1);

    /* 启动 TIM1 连续输出 PWM */
    LHL_TIM_Start(pTIM1);
}

/**
  * @brief  运行期间动态修改 P0.5 (TIM1_CH1) PWM 占空比
  * @param  duty_percent: 占空比百分比（0 ~ 100）
  * @retval None
  */
void P05_PWM_SetDuty(uint8_t duty_percent)
{
    uint16_t period;
    uint16_t pulse;

    /* 从 TIM1_ARR 读取当前周期配置 */
    period = (uint16_t)(pTIM1->ARR & TIMER_TIMX_ARR_ARR_Msk);
    pulse  = P05_PWM_CalcPulse(period, duty_percent);

    /* 更新 CCR1 预装载寄存器，将在下一个更新事件（UEV）平滑生效 */
    LHL_TIM_PWM_SetDutyCycle(pTIM1, pulse, TIM_CHANNEL_1);
}


/* 使用内部HSI时钟，将HCLK/APB时钟统一配置到4MHz */
void SystemClock_Config(void)
{
    RCC_ClockInitTypeDef Clock_InitStructure;

    Clock_InitStructure.SYSCLKSource = RCC_SYSCLK_SRC_HSI_DIV2; /* 32MHz HSI / 2 = 16MHz */
    Clock_InitStructure.AHBCLKDivider = RCC_SYSCLK_DIV4;        /* HCLK = 16MHz / 4 = 4MHz */
    Clock_InitStructure.APB1CLKDivider = RCC_HCLK_DIV1;         /* PCLK1 = HCLK = 4MHz */
    Clock_InitStructure.APB2CLKDivider = RCC_HCLK_DIV1;         /* PCLK2 = HCLK = 4MHz */

    if (LHL_RCC_ClockConfig(&Clock_InitStructure, LHL_NRM_TIMEOUT) != LHL_OK)
    {
        while(1)
        {
        }
    }
}

/* 配置模板默认LED引脚：P0.0 */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = TEMPLATE_LED_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Current = GPIO_CURRENT_4mA;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.SchmittTrigger = DISABLE;
    GPIO_InitStructure.Alternate = GPIOx_AF_GPIO;

    LHL_GPIO_Init(TEMPLATE_LED_PORT, &GPIO_InitStructure);
}

/*********************************End of File**********************************/
