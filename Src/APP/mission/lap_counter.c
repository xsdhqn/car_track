#include "lap_counter.h"
#include "../trace/trace_control.h"
/* ===================== 可调参数 ===================== */
#define APOINT_DEBOUNCE         2       /* 2 次连续全黑才认为过黑点 */
#define LAP_TARGET              2       /* 目标圈数 */
#define CROSSES_PER_LAP         4       /* 每 4 个黑点计 1 圈 */

/* ===================== 静态变量 ===================== */
static u8 s_debounce = 0;
static u8 s_a_left = 0;            /* 已离开黑点标志 */
static u8 s_cross_count = 0;       /* 当前圈内黑点计数 (0~3) */
static u8 s_lap_count = 0;
static u8 s_just_passed = 0;       /* 本周期刚过 A 点（第 4 个黑点） */
static u8 s_just_passed_cross = 0; /* 本周期刚过黑点（任意） */

/* ===================== 接口实现 ===================== */

void lap_counter_init(void)
{
    s_debounce = 0;
    s_a_left = 1;        /* 初始即认为已离开黑点，可直接检测第一个黑点 */
    s_cross_count = 0;
    s_lap_count = 0;
    s_just_passed = 0;
    s_just_passed_cross = 0;
}

void lap_counter_update(void)
{
    s_just_passed = 0;
    s_just_passed_cross = 0;

    if (!trace_control_is_all_black())
    {
        /* 未检测到黑点（非全黑），清零消抖并标记已离开黑点 */
        s_debounce = 0;
        if (!s_a_left)
        {
            s_a_left = 1;   /* 第一次离开黑点区域，允许下一次检测 */
        }
        return;
    }

    /* 连续检测到全黑（十字黑点），消抖计数 */
    if (s_debounce < APOINT_DEBOUNCE)
    {
        s_debounce++;
        return;
    }

    /* 消抖完成（连续 N 次全黑），但仍需确认已离开过黑点才算新触发 */
    if (!s_a_left)
    {
        return;
    }

    /* 通过黑点：离开过 → 再次到达全黑消抖通过 */
    s_a_left = 0;
    s_cross_count++;
    s_just_passed_cross = 1;   /* 每个黑点都触发 */

    if (s_cross_count >= CROSSES_PER_LAP)
    {
        /* 第 4 个黑点 = A 点，计一圈 */
        s_cross_count = 0;
        s_lap_count++;
        s_just_passed = 1;
    }
}

u8 lap_counter_just_passed_cross(void)
{
    return s_just_passed_cross;
}

u8 lap_counter_just_passed_a(void)
{
    return s_just_passed;
}

u8 lap_counter_get_laps(void)
{
    return s_lap_count;
}

u8 lap_counter_is_finished(void)
{
    return (s_lap_count >= LAP_TARGET);
}

u8 lap_counter_get_target(void)
{
    return LAP_TARGET;
}

void lap_counter_reset(void)
{
    s_debounce = 0;
    s_a_left = 1;        /* 初始即认为已离开黑点，可直接检测第一个黑点 */
    s_cross_count = 0;
    s_lap_count = 0;
    s_just_passed = 0;
    s_just_passed_cross = 0;
}
