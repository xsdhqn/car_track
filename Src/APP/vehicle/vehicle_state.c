#include "vehicle_state.h"
#include "../../BSP/sensor/encoder.h"

/* 每总脉冲对应的距离（mm * 100 / pulse）
 * 分母乘 2 是因为 encoder_read() 返回左右轮脉冲之和
 * 使用 314/100 近似 PI */
#define VEHICLE_DIST_PER_PULSE_X100 \
    ((u32)((314UL * VEHICLE_WHEEL_DIAMETER_MM) / (VEHICLE_ENCODER_PPR * 2)))

/* 速度一阶低通滤波：filtered += (raw - filtered) >> N
 * N=3 → 平滑系数 1/8，100ms 采样周期下时间常数约 700ms，
 * 有效抑制振动尖峰，同时对真实速度变化保持亚秒级响应。 */
#define VEHICLE_SPEED_LPF_SHIFT  3

/* 静态变量 */
static u32 s_distance_cm = 0;
static u32 s_distance_acc = 0;  /* 余数累积器：单位与 dist_inc_x100 相同 (mm×100)，进位后保留 */
static u16 s_speed_cm_s = 0;
static u16 s_speed_raw = 0;            /* 滤波前的原始速度，调试用（暂存） */

void vehicle_init(void)
{
    s_distance_cm = 0;
    s_distance_acc = 0;
    s_speed_cm_s = 0;
    s_speed_raw  = 0;
    encoder_read();  /* 清空编码器初始化期间可能积累的噪声脉冲 */
}

void vehicle_update(void)
{
    int pulses = encoder_read();  /* 取上次调用至今的累计脉冲 */
    u32 dist_inc_x100;

    /* 过滤噪声：负值（方向反转，当前硬件不支持）或孤立脉冲一律丢弃。
     * 阈值 2 意味着 100ms 采样周期内至少需要 2 个有效脉冲，
     * 配合 ISR 消抖可彻底消除振动产生的虚假里程。 */
    if (pulses < 2)
    {
        pulses = 0;
    }

    /* 距离增量：mm * 100 / pulse
     * dist_inc_x100 单位是 mm×100 (即 0.01mm)
     * 1 cm = 10 mm = 1000 × 0.01mm，所以除以 1000 得到 cm */
    dist_inc_x100 = (u32)pulses * VEHICLE_DIST_PER_PULSE_X100;

    /* 用同单位累加器避免每次整除丢余数：满 1000 进位 1cm，余数保留 */
    s_distance_acc += dist_inc_x100;
    s_distance_cm += s_distance_acc / 1000;   /* 0.01mm → cm: /1000 */
    s_distance_acc %= 1000;                   /* 保留不足 1cm 的尾数 */

    /* 速度(cm/s) = pulses * dist_per_pulse_x100 / sample_ms */
    s_speed_raw = (u16)((u32)pulses * VEHICLE_DIST_PER_PULSE_X100 / VEHICLE_SAMPLE_MS);

    /* 一阶低通滤波抑制振动尖峰：
     * filtered += (raw - filtered) >> SHIFT
     * 首次有效读数直接赋值，避免从 0 缓慢爬升 */
    if (s_speed_cm_s == 0 && s_speed_raw != 0)
    {
        s_speed_cm_s = s_speed_raw;
    }
    else
    {
        s_speed_cm_s += (s16)(s_speed_raw - s_speed_cm_s) >> VEHICLE_SPEED_LPF_SHIFT;
    }
}

u16 vehicle_get_speed_cm_s(void)
{
    return s_speed_cm_s;
}

u32 vehicle_get_distance_cm(void)
{
    return s_distance_cm;
}

void vehicle_reset_distance(void)
{
    s_distance_cm = 0;
    s_distance_acc = 0;
}
