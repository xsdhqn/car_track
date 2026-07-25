#ifndef __LAP_COUNTER_H
#define __LAP_COUNTER_H

#include "stm32f10x.h"

/* 初始化计圈模块 */
void lap_counter_init(void);

/* 周期性更新 A 点检测状态 */
void lap_counter_update(void);

/* 是否刚刚经过黑点（任意黑点，本次调用周期内为真，仅一次） */
u8 lap_counter_just_passed_cross(void);

/* 是否刚刚经过 A 点（第 4 个黑点，本次调用周期内为真，仅一次） */
u8 lap_counter_just_passed_a(void);

/* 获取当前已跑圈数 */
u8 lap_counter_get_laps(void);

/* 是否已完成 2 圈 */
u8 lap_counter_is_finished(void);

/* 获取目标圈数 */
u8 lap_counter_get_target(void);

/* 重置圈数 */
void lap_counter_reset(void);

#endif
