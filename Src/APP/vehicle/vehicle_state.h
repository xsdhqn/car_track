#ifndef __VEHICLE_STATE_H
#define __VEHICLE_STATE_H

#include "stm32f10x.h"

/* 物理参数：根据实际小车修改 */
/* 轮胎直径：参数表给出 6.6cm，故取 66mm */
#define VEHICLE_WHEEL_DIAMETER_MM   66  /* 驱动轮直径，单位 mm */

/* 编码器每转脉冲数（单轮）。
w * 注意：若编码器安装在电机轴（减速前），实际单轮 PPR = 电机轴 PPR × 减速比(48)。
 * 例如常见 6/13/20  counts/电机转的霍尔编码器，对应单轮 PPR 约为 288/624/960。
 * 当前值 20 仅在编码器直接安装在输出轮上时成立，强烈建议上板后实测校准：
 *   方法1：用手缓慢转一圈车轮，通过串口打印 encoder_read() 累计脉冲数；
 *   方法2：让小车直线行驶 100cm，比较 vehicle_get_distance_cm() 与实际距离，反推 PPR。 */
#define VEHICLE_ENCODER_PPR         40 /*  编码器每转脉冲数（单轮），需实测校准 */

/* 采样周期（ms），应与调用 vehicle_update() 的周期一致 */
#define VEHICLE_SAMPLE_MS           100

/* 初始化车速/里程状态 */
void vehicle_init(void);

/* 周期性读取编码器并更新速度、里程；建议每 VEHICLE_SAMPLE_MS 调用一次 */
void vehicle_update(void);

/* 获取当前速度，单位 cm/s */
u16 vehicle_get_speed_cm_s(void);

/* 获取累计单次里程，单位 cm */
u32 vehicle_get_distance_cm(void);

/* 重置累计里程 */
void vehicle_reset_distance(void);

#endif
