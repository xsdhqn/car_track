#ifndef __INFRARED_H
#define __INFRARED_H
#include "stm32f10x.h"

#define TRACE_CH_COUNT  4

void Infrared_Init(void);
uint16_t Infrared_GetADC(uint8_t ch);  // ch: ADC 通道号 (2/3/6/7)
#endif
