#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "PWM.h"
#include "CAR.h"
#include "Serial.h"
#include "Servo.h"
#include "Ultrasound.h"
#include "Track.h"

uint16_t Data1;

/*  施密特阈值 — 抑制黑白边界抖动
    黑线 ADC > TH_HIGH → 判黑  (实测黑线 800~1000+)
    白底 ADC < TH_LOW  → 判白  (实测白底 几百)
    中间区域保持上一次状态不翻转                                */
#define TH_HIGH  700
#define TH_LOW   450

/*  传感器布局（等距，4 路）
    最右 R2=PA2   右 R1=PA3   左 L1=PA6   最左 L2=PA7          */

int main(void)
{
    Car_Init();
    Serial_Init();
    Servo_Init();
    Ultrasound_Init();
    Infrared_Init();

    uint8_t R2 = 0, R1 = 0, L1 = 0, L2 = 0;   // 0=白底, 1=黑线

    while (1)
    {
        // ---- 读取 4 路 ADC ----
        uint16_t aR2 = Infrared_GetADC(2);     // PA2 — 最右
        uint16_t aR1 = Infrared_GetADC(3);     // PA3 — 右
        uint16_t aL1 = Infrared_GetADC(6);     // PA6 — 左
        uint16_t aL2 = Infrared_GetADC(7);     // PA7 — 最左

        // ---- 施密特触发器 ----
        if (R2) { if (aR2 < TH_LOW)  R2 = 0; } else { if (aR2 > TH_HIGH) R2 = 1; }
        if (R1) { if (aR1 < TH_LOW)  R1 = 0; } else { if (aR1 > TH_HIGH) R1 = 1; }
        if (L1) { if (aL1 < TH_LOW)  L1 = 0; } else { if (aL1 > TH_HIGH) L1 = 1; }
        if (L2) { if (aL2 < TH_LOW)  L2 = 0; } else { if (aL2 > TH_HIGH) L2 = 1; }

        // ---- 4 路循迹判断 ----
        if (L2 && L1 && R1 && R2)               // 4个全在黑线 → 直行
        {
            Go_Ahead();
        }
        else if (!L2 && !L1 && !R1 && !R2)      // 4个全在白底 → 停车
        {
            Car_Stop();
        }
        else if (L1 && R1)                      // 中间2个在线 → 直行
        {
            Go_Ahead();
        }
        else if (L1 && !R1)                     // 仅左侧在线 → 右偏，左转
        {
            Turn_Left();
        }
        else if (!L1 && R1)                     // 仅右侧在线 → 左偏，右转
        {
            Turn_Right();
        }
        else if (L2 && !R2)                     // 最左在线、最右不在 → 大幅左转
        {
            Self_Left();
        }
        else if (!L2 && R2)                     // 最右在线、最左不在 → 大幅右转
        {
            Self_Right();
        }
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        Data1 = USART_ReceiveData(USART1);
        if      (Data1 == 0x30) Car_Stop();
        if      (Data1 == 0x31) Go_Ahead();
        if      (Data1 == 0x32) Go_Back();
        if      (Data1 == 0x33) Turn_Left();
        if      (Data1 == 0x34) Turn_Right();
        if      (Data1 == 0x35) Self_Left();
        if      (Data1 == 0x36) Self_Right();
        if      (Data1 == 0x37) Servo_SetAngle(0);
        if      (Data1 == 0x38) Servo_SetAngle(90);
        if      (Data1 == 0x39) Servo_SetAngle(180);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
