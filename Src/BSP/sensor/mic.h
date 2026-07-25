#ifndef __MIC_H
#define __MIC_H

#include "stm32f10x.h"
#include "config.h"

/* ADC 阈值：ADC 值低于此阈值判定为有声音（电压拉低）。
 * VREF=3.3V, 12bit ADC: 2.2V ≈ 2730
 * 实测：没声音 2.3~2.5V(2854~3102)，有声音时电压降低。
 * 阈值设 2.2V，确保低于没声音下限 2.3V，高于有声音电压。 */
#define MIC_ADC_THRESHOLD       2800

/* 初始化咪头检测接口（PB0 ADC1_CH8 模拟输入） */
void mic_init(void);

/* 扫描咪头状态（非阻塞消抖，ADC 阈值判断）。
 * 返回值：0 = 无信号，1 = 单次拍手，2 = 连续两次拍手。
 * 建议调用周期：10~20ms */
u8   mic_scan(void);

/* 获取 ADC 原始值（用于调试观察），范围 0~4095 */
u16  mic_get_raw(void);

/* 声控功能使能/禁用控制。
 * 后续 UI 可通过此接口绑定开关。 */
void mic_set_enabled(u8 enabled);
u8   mic_is_enabled(void);

#endif /* __MIC_H */
