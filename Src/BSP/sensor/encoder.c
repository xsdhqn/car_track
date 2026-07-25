#include "encoder.h"
#include "key.h"
#include <stddef.h>

/* ========================================================================
 * 编码器配置说明：
 *
 * 当前小车采用同轴双轮驱动方案，左右轮直接同轴连接电机，
 * 不存在差速比（differential ratio = 1:1）。
 *
 * 因此 encoder_read() 返回值为左右轮脉冲之和，
 * 用于统一计算车速和里程。
 *
 * 低通滤波：采用一阶 IIR 滤波器平滑编码器读数，
 * 抑制机械振动和电磁干扰引起的高频噪声。
 * ======================================================================== */

/* ========================================================================
 * DWT (Data Watchpoint and Trace) 周期计数器
 *
 * 用于编码器中断消抖的时间戳。DWT->CYCCNT 是 Cortex-M3 内核自带的
 * 32 位自由运行计数器，每 CPU 时钟周期 +1（72MHz 时约 59.6 秒回绕）。
 *
 * STM32F10x 标准外设库的头文件未提供 DWT 寄存器定义，此处手动声明。
 * ======================================================================== */
#define DWT_CTRL            (*(volatile u32 *)0xE0001000UL)
#define DWT_CYCCNT          (*(volatile u32 *)0xE0001004UL)
#define DWT_CTRL_CYCCNTENA  (1UL << 0)

#define DEMCR               (*(volatile u32 *)0xE000EDFCUL)
#define DEMCR_TRCENA        (1UL << 24)

/* 消抖窗口（μs）：连续两个上升沿间隔必须 > 此值才算有效脉冲 */
#define ENC_DEBOUNCE_US     500

static volatile s32 enc_left_pulse = 0;
static volatile s32 enc_right_pulse = 0;
static volatile s32 enc_left_total = 0;   /* 上电以来累计，不清零 */
static volatile s32 enc_right_total = 0;  /* 上电以来累计，不清零 */

/* 消抖用：左右轮各自上次有效脉冲的 DWT_CYCCNT 值 */
static volatile u32 enc_l_last_cycles = 0;
static volatile u32 enc_r_last_cycles = 0;

/* 消抖阈值：初始化时根据 SystemCoreClock 换算成 CPU 周期数 */
static u32 enc_debounce_cycles = 0;

/* ========================================================================
 * 低通滤波器参数（一阶 IIR）
 *
 * 滤波公式：y[n] = α * x[n] + (1 - α) * y[n-1]
 *
 * ENC_LPF_ALPHA: 滤波系数，范围 (0, 1]
 *   - 值越小，滤波越强，响应越慢
 *   - 值越大，滤波越弱，响应越快
 *   - 推荐值：0.1 ~ 0.3（可根据实际噪声情况调整）
 *
 * enc_lpf_prev: 上一次滤波输出值，用于递推计算
 * ======================================================================== */
#define ENC_LPF_ALPHA        0.2f    /* 滤波系数，可调 */

static float enc_lpf_prev = 0.0f;    /* 滤波器状态变量 */

void encoder_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_ENC_CLK_ALL, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_ENC_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_ENC_L_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_ENC_R_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_ENC_R_PORT, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

    /* 先清除 GPIO/EXTI 配置过程中可能产生的挂起中断 */
    EXTI_ClearITPendingBit(BSP_ENC_L_EXTI);
    EXTI_ClearITPendingBit(BSP_ENC_R_EXTI);

    EXTI_InitStructure.EXTI_Line = BSP_ENC_L_EXTI | BSP_ENC_R_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* DWT 消抖必须早于 NVIC 使能 —— 确保首批中断就受到保护 */
    DEMCR |= DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
    enc_debounce_cycles = (SystemCoreClock / 1000000UL) * ENC_DEBOUNCE_US;

    /* 显式清零脉冲计数、消抖时间戳和滤波器状态，确保从干净状态启动 */
    enc_left_pulse = 0;
    enc_right_pulse = 0;
    enc_l_last_cycles = 0;
    enc_r_last_cycles = 0;
    enc_lpf_prev = 0.0f;   /* 初始化滤波器输出为 0 */

    /* 注意：EXTI9_5_IRQn 与按键模式键共享，最终优先级由 key_start_irq_init 统一设置 */
    NVIC_InitStructure.NVIC_IRQChannel = BSP_ENC_L_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void encoder_left_inc(void)
{
    enc_left_pulse++;
    enc_left_total++;
}

void encoder_right_inc(void)
{
    enc_right_pulse++;
    enc_right_total++;
}

int encoder_read(void)
{
    int raw_val;
    float filtered_val;

    /* 读数和清零需原子执行，防止中断期间丢脉冲 */
    __disable_irq();
    raw_val = (int)(enc_left_pulse + enc_right_pulse);
    enc_left_pulse = 0;
    enc_right_pulse = 0;
    __enable_irq();

    /* 应用一阶 IIR 低通滤波器
     * y[n] = α * x[n] + (1 - α) * y[n-1]
     * 有效抑制编码器高频噪声（机械振动、电磁干扰等）*/
    filtered_val = ENC_LPF_ALPHA * (float)raw_val + (1.0f - ENC_LPF_ALPHA) * enc_lpf_prev;
    enc_lpf_prev = filtered_val;

    /* 四舍五入（而非截断）：避免每次调用系统性丢失 ~0.5 脉冲，
     * 误差通过 IIR 状态变量 enc_lpf_prev 在后续周期自动补偿。 */
    return (int)(filtered_val + 0.5f);
}

void encoder_get_counts(s32 *left, s32 *right)
{
    __disable_irq();
    if (left != NULL)
    {
        *left = enc_left_total;
    }
    if (right != NULL)
    {
        *right = enc_right_total;
    }
    __enable_irq();
}

/* 仅供调试页使用：清零累计总脉冲，开始新一轮测量 */
void encoder_reset_totals(void)
{
    __disable_irq();
    enc_left_total = 0;
    enc_right_total = 0;
    __enable_irq();
}

void EXTI9_5_IRQHandler(void)
{
    u32 now = DWT_CYCCNT;

    if (EXTI_GetITStatus(BSP_ENC_L_EXTI) != RESET)
    {
        /* 消抖：与上次有效脉冲的间隔必须 > 消抖窗口，否则视为毛刺丢弃 */
        if (now - enc_l_last_cycles > enc_debounce_cycles)
        {
            encoder_left_inc();
            enc_l_last_cycles = now;
        }
        EXTI_ClearITPendingBit(BSP_ENC_L_EXTI);
    }
    if (EXTI_GetITStatus(BSP_ENC_R_EXTI) != RESET)
    {
        if (now - enc_r_last_cycles > enc_debounce_cycles)
        {
            encoder_right_inc();
            enc_r_last_cycles = now;
        }
        EXTI_ClearITPendingBit(BSP_ENC_R_EXTI);
    }
    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        key_start_irq_handler();
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
