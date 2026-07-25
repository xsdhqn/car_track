#include "mic.h"
#include "../system/board.h"
#include "../../APP/app.h"

/* ===================== 可调参数 ===================== */
/* 消抖阈值：连续 MIC_DEBOUNCE_CNT 次 (10ms/次) ADC 低于阈值认为有效声控信号。
 * 值越大抗干扰越强，但响应越慢。建议 2~5。 */
#define MIC_DEBOUNCE_CNT        3       /* 3 * 10ms = 30ms 消抖 */

/* 连续拍手识别时间窗口：两次有效信号间隔不超过此值视为连续拍手。
 * 设为 0 禁用连续拍手检测，每次有效信号都触发。 */
#define MIC_DOUBLE_TAP_MS       800     /* 800ms 内两次拍手 */

/* ===================== 内部状态 ===================== */
typedef enum {
    MIC_ST_IDLE,            /* 空闲，等待第一次有效信号 */
    MIC_ST_DEBOUNCE_1,      /* 第一次信号消抖中 */
    MIC_ST_WAIT_SECOND,     /* 等待第二次拍手（如果启用双击模式） */
    MIC_ST_DEBOUNCE_2       /* 第二次信号消抖中 */
} mic_state_t;

static mic_state_t s_state = MIC_ST_IDLE;
static u8           s_debounce_cnt = 0;
static u16          s_last_trigger_tick = 0;   /* 上次触发的系统 tick */
static u8           s_enabled = 1;             /* 声控功能使能标志，默认开启 */
static u16          s_last_adc = 0;            /* 最近一次 ADC 读数，供调试 */
static u8           s_adc_ready = 0;           /* ADC1 是否就绪 */

/* ===================== 接口实现 ===================== */

void mic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_MIC_CLK, ENABLE);

    /* PB0 配置为模拟输入（ADC1_CH8）。
     * 读取 LM393 比较器输出电压，用软件阈值区分有/无声音。
     * 实测：没声音 ≈ 2.3~2.5V(ADC 2854~3102)，有声音时电压降低。
     * 检测逻辑：ADC 值低于阈值 → 有声音（低电平有效）。 */
    GPIO_InitStructure.GPIO_Pin  = BSP_MIC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(BSP_MIC_PORT, &GPIO_InitStructure);

    s_state = MIC_ST_IDLE;
    s_debounce_cnt = 0;
    s_last_trigger_tick = 0;
    s_last_adc = 0;
    s_adc_ready = 0;
}

/* 确保 ADC1 已上电并使能（幂等，trace_init 调用后无需额外操作） */
static void mic_adc_ensure_ready(void)
{
    if (!s_adc_ready)
    {
        /* trace_init() 已初始化 ADC1，这里只补使能和校准（幂等安全） */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

        ADC_Cmd(ADC1, ENABLE);

        ADC_ResetCalibration(ADC1);
        while (ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);
        while (ADC_GetCalibrationStatus(ADC1));

        s_adc_ready = 1;
    }
}

/* 软件触发单次 ADC 转换，读取指定通道。
 * 与 trace.c 的 adc_get_ch 逻辑相同但独立实现，避免模块间耦合。 */
static u16 mic_adc_read_ch(u8 ch)
{
    /* 先清 EOC，避免读到上一次转换的残留值 */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    /* 配置规则通道：单通道，239.5 周期采样 */
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);

    /* 软件触发一次转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* 等待转换完成 */
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    /* 读结果（读 DR 会自动清 EOC） */
    return ADC_GetConversionValue(ADC1);
}

u16 mic_get_raw(void)
{
    mic_adc_ensure_ready();
    s_last_adc = mic_adc_read_ch(BSP_MIC_ADC_CH);
    return s_last_adc;
}

/* 读取一次 ADC 并与阈值比较，返回 0/1。
 * 有声音时电压降低，ADC 低于阈值 → 返回 1（有效）。 */
static u8 mic_read_level(void)
{
    mic_adc_ensure_ready();
    s_last_adc = mic_adc_read_ch(BSP_MIC_ADC_CH);
    return (s_last_adc < MIC_ADC_THRESHOLD) ? 1 : 0;
}

/* 状态机扫描函数，由 app_update() 每 10ms 调用一次。
 * 返回值：0 = 无触发，1 = 单次拍手，2 = 连续两次拍手。
 * 当 MIC_DOUBLE_TAP_MS > 0 时，第一次拍手后进入等待窗口；
 * 窗口内收到第二次拍手返回 2，窗口超时返回 1。 */
u8 mic_scan(void)
{
    u32 now;
    u16 elapsed;
    u8 result = 0;
    u8 level = mic_read_level();  /* ADC 低于阈值=有声音(电压拉低) */

    switch (s_state)
    {
        case MIC_ST_IDLE:
            if (level)  /* ADC 低于阈值(电压拉低)：有声音 */
            {
                s_state = MIC_ST_DEBOUNCE_1;
                s_debounce_cnt = 1;
            }
            break;

        case MIC_ST_DEBOUNCE_1:
            if (level)  /* 低电平持续，继续消抖计数 */
            {
                s_debounce_cnt++;
                if (s_debounce_cnt >= MIC_DEBOUNCE_CNT)
                {
                    /* 第一次信号消抖完成 */
#if MIC_DOUBLE_TAP_MS > 0
                    s_state = MIC_ST_WAIT_SECOND;
                    s_last_trigger_tick = (u16)(app_get_tick() & 0xFFFF);
#else
                    /* 单击模式：立即返回单次触发 */
                    s_state = MIC_ST_IDLE;
                    result = 1;
#endif
                    s_debounce_cnt = 0;
                }
            }
            else
            {
                /* 信号消失，回退到空闲 */
                s_state = MIC_ST_IDLE;
                s_debounce_cnt = 0;
            }
            break;

        case MIC_ST_WAIT_SECOND:
            /* 计算距上次触发的时长 */
            now = app_get_tick();
            if (now >= s_last_trigger_tick)
            {
                elapsed = (u16)(now - s_last_trigger_tick);
            }
            else
            {
                /* tick 溢出处理（16 位溢出约 65.5s，几乎不会发生） */
                elapsed = 0xFFFF - (u16)(s_last_trigger_tick - now);
            }

            if (level)
            {
                /* 在时间窗口内检测到新的信号，开始第二次消抖 */
                s_state = MIC_ST_DEBOUNCE_2;
                s_debounce_cnt = 1;
            }
            else if (elapsed > MIC_DOUBLE_TAP_MS)
            {
                /* 超时未收到第二次拍手，判定为单次拍手 */
                s_state = MIC_ST_IDLE;
                s_debounce_cnt = 0;
                result = 1;
            }
            break;

        case MIC_ST_DEBOUNCE_2:
            if (level)  /* 第二次信号持续，继续消抖 */
            {
                s_debounce_cnt++;
                if (s_debounce_cnt >= MIC_DEBOUNCE_CNT)
                {
                    /* 第二次信号消抖完成，判定为连续拍手 */
                    s_state = MIC_ST_IDLE;
                    s_debounce_cnt = 0;
                    result = 2;
                }
            }
            else
            {
                /* 第二次信号中断，回到等待窗口 */
                s_state = MIC_ST_WAIT_SECOND;
                s_debounce_cnt = 0;
            }
            break;

        default:
            s_state = MIC_ST_IDLE;
            s_debounce_cnt = 0;
            break;
    }

    return result;
}

void mic_set_enabled(u8 enabled)
{
    s_enabled = enabled ? 1 : 0;
    if (!s_enabled)
    {
        /* 禁用时重置状态机，避免残留状态 */
        s_state = MIC_ST_IDLE;
        s_debounce_cnt = 0;
    }
}

u8 mic_is_enabled(void)
{
    return s_enabled;
}
