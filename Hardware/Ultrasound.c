#include "stm32f10x.h"
#include "Delay.h"

/*
  超声波自搭方案（TCT40-16T/R + LM324 放大 + LM393 比较）
    发射: PA8  → TIM1_CH1  → 40kHz PWM → 外部三极管驱动探头
    接收: PB6  → TIM4_CH1  → 输入捕获（飞行时间法，测第一个上升沿）

  检测窗口（避障 20~50cm）:
    发射结束后 0.5~3.0ms → 对应约 8.6~51.7cm
    d = t/58,  t = 58×d

  原理:
    1. 发射 500µs 的 40kHz 脉冲
    2. 盲区 250µs（发射余震 + 电气串扰），忽略此期间的中断
    3. 盲区结束后 TIM4（1µs 精度）开始计时
    4. 捕获 PB6 第一个上升沿 → 飞行时间 = 盲区 + TIM4值
    5. 距离(cm) = 飞行时间(µs) / 58

  注意: LM393 比较器输出的是 40kHz 方波串（不是单脉冲）
        所以不能测脉宽，只能测飞行时间（第一个上升沿）           */

#define TX_US     500   // 发射时长（µs）
#define BLIND_US  250   // 盲区（µs），跳开发射余震+串扰，对应 ≈4cm
#define TO_US    3500   // TIM4 超时（µs），对应 ≈65cm，超出避障需求 50cm 有余量

static volatile uint16_t us_echo_us = 0;  // 回波到达时刻（µs），从盲区结束开始算
static volatile uint8_t  us_ok     = 0;   // 0=无回波, 1=已捕获
static volatile uint8_t  us_listen = 0;   // 0=盲区忽略, 1=允许捕获

void Ultrasound_Init(void)
{
    /* ---- 时钟 ---- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_GPIOB | RCC_APB1Periph_TIM4, ENABLE);

    /* ==== 发射: PA8 → TIM1_CH1, 40kHz PWM, 50% 占空比 ==== */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM1);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period            = 1800 - 1;  // 72M/1800 = 40kHz
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 899;       // 900/1800 = 50%
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);   // 高级定时器 MOE

    /* ==== 接收: PB6 → TIM4_CH1, 输入捕获, 1µs 分辨率 ==== */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     // 下拉 → 无回波时稳定低
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 72M / 72 = 1MHz → 1µs 精度，ARR=5000 覆盖 5ms（≈86cm，远超需求 50cm）
    TIM_InternalClockConfig(TIM4);
    TIM_TimeBaseInitStructure.TIM_Period            = 5000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;   // 只抓上升沿
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x0F;                     // 数字滤波（滤毛刺）
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);              // TIM4 持续运行，测距前清计数器
}

/*  返回距离（cm），超时或无回波返回 0   */
float Test_Distance(void)
{
    us_listen  = 0;
    us_ok      = 0;
    us_echo_us = 0;

    /* 1. 发射 */
    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);
    Delay_us(TX_US);
    TIM_Cmd(TIM1, DISABLE);

    /* 2. 盲区：跳开发射余震 + 电气串扰 */
    Delay_us(BLIND_US);

    /* 3. 计时起点：清 TIM4 → 开放捕获 */
    TIM_SetCounter(TIM4, 0);
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    us_listen = 1;

    /* 4. 等回波，超时 ≈3500µs → 约 65cm 量程 */
    uint32_t to = TO_US;
    while (!us_ok && --to) {}

    us_listen = 0;

    /* 5. 飞行时间 = 盲区 + TIM4值，换算为 cm */
    if (us_ok)
        return (BLIND_US + us_echo_us) / 58.0f;
    else
        return 0.0f;
}

/*  TIM4 捕获中断 —— 只在 us_listen=1 时接受            */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET)
    {
        if (us_listen)
        {
            us_echo_us = TIM_GetCapture1(TIM4);
            us_ok      = 1;
        }
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    }
}
