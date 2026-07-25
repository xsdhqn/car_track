#include "ui_debug.h"
#include "ui_dashboard.h"
#include "../app.h"
#include "../../BSP/display/oled_spi.h"
#include "../../BSP/display/oledfont.h"
#include "../control/app_fsm.h"
#include "../trace/trace_control.h"
#include "../obstacle/obstacle_guard.h"
#include "../vehicle/vehicle_state.h"
#include "../mission/lap_counter.h"
#include "../motion/motion_control.h"
#include "../../BSP/actuator/buzzer.h"
#include "../../BSP/actuator/led.h"
#include "../../BSP/sensor/trace.h"
#include "../../BSP/sensor/encoder.h"
#include "../../BSP/sensor/ultrasonic.h"
#include "../../BSP/sensor/mic.h"
#include "../../BSP/input/key.h"
#include <stdio.h>
#include <string.h>

/* ===================== 编码器 PPR 测量页 ===================== */
/* 用户流程：KEY1 清零 → 转轮子 1 圈 → 看 PPR 值 */
#define ENC_MEASURE_MAX      3     /* 最多保存 3 次测量求平均 */

static s32  s_enc_meas[ENC_MEASURE_MAX][2];  /* [n][0]=左 [n][1]=右 */
static u8   s_enc_meas_cnt = 0;              /* 已完成测量次数 */
static s32  s_enc_meas_locked[2] = {0, 0};   /* 当前锁定的 PPR */

static void ui_debug_enter_encoder(void)
{
    u8 i;
    for (i = 0; i < ENC_MEASURE_MAX; i++)
    {
        s_enc_meas[i][0] = 0;
        s_enc_meas[i][1] = 0;
    }
    s_enc_meas_cnt = 0;
    s_enc_meas_locked[0] = 0;
    s_enc_meas_locked[1] = 0;
    encoder_reset_totals();
}

static void ui_debug_draw_encoder(void)
{
    char buf[36];
    s32 left = 0, right = 0;

    encoder_get_counts(&left, &right);

    /* y=1: 实时脉冲计数 */
    sprintf(buf, "L:+%05ld R:+%05ld", left, right);
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    /* y=2: 操作提示 */
    if (left == 0 && right == 0)
    {
        oled_spi_show_string(0, 2, (u8 *)"<< TURN 1 REV >>", 8);
    }
    else
    {
        sprintf(buf, "PPR L:%5ld R:%5ld", left, right);
        oled_spi_show_string(0, 2, (u8 *)buf, 8);
    }

    /* y=3: 锁定的 PPR 值 */
    sprintf(buf, "LOCK: L:%5ld R:%5ld", s_enc_meas_locked[0], s_enc_meas_locked[1]);
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    /* y=4: 多次测量记录 */
    {
        char m1[8] = "--", m2[8] = "--", m3[8] = "--";
        if (s_enc_meas_cnt >= 1) sprintf(m1, "%ld/%ld", s_enc_meas[0][0], s_enc_meas[0][1]);
        if (s_enc_meas_cnt >= 2) sprintf(m2, "%ld/%ld", s_enc_meas[1][0], s_enc_meas[1][1]);
        if (s_enc_meas_cnt >= 3) sprintf(m3, "%ld/%ld", s_enc_meas[2][0], s_enc_meas[2][1]);
        sprintf(buf, "M1:%-7s M2:%-7s", m1, m2);
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
        if (s_enc_meas_cnt >= 3)
        {
            sprintf(buf, "M3:%-7s", m3);
            oled_spi_show_string(30, 5, (u8 *)buf, 8);
        }
    }

    /* y=5~6: 按键说明 */
    {
        s32 avg_l = 0, avg_r = 0;
        u8 i;
        if (s_enc_meas_cnt > 0)
        {
            for (i = 0; i < s_enc_meas_cnt; i++)
            {
                avg_l += s_enc_meas[i][0];
                avg_r += s_enc_meas[i][1];
            }
            avg_l /= s_enc_meas_cnt;
            avg_r /= s_enc_meas_cnt;
            sprintf(buf, "AVG: L:%4ld R:%4ld (%uX)", avg_l, avg_r, s_enc_meas_cnt);
            oled_spi_show_string(0, 6, (u8 *)buf, 8);
        }
        else
        {
            oled_spi_show_string(0, 5, (u8 *)"KEY1:ZERO  KEY2:LOCK", 8);
            oled_spi_show_string(0, 6, (u8 *)"HOLD 2s -> EXIT", 8);
        }
    }

    /* 补一行 KEY 提示（当有测量记录时 y=5 被平均值占用） */
    if (s_enc_meas_cnt > 0)
    {
        oled_spi_show_string(0, 5, (u8 *)"KEY1:ZERO  KEY2:LOCK", 8);
    }
}

/* 外部报警 LED + 蜂鸣器自检周期 */
#define UI_ACT_LED_PERIOD_TICKS     (500 / UI_TASK_PERIOD_MS)
#define UI_ACT_BEEP_PERIOD_TICKS    (1000 / UI_TASK_PERIOD_MS)

/* 电机调试参数：直线前进 */
#define UI_MOTOR_FWD_SPEED_PCT     80      /* 默认前进速度 80% */
#define UI_MOTOR_FWD_SPEED_MIN     20      /* 最小速度 20% */
#define UI_MOTOR_FWD_SPEED_MAX     100     /* 最大速度 100% */
#define UI_MOTOR_FWD_SPEED_STEP    5       /* 速度调节步进 5% */

/* 电机补偿标定参数：固定速度直行 2 秒，根据编码器脉冲差计算建议增益 */
#define UI_MOTOR_CALIB_SPEED_PCT    60
#define UI_MOTOR_CALIB_TICKS        (2000 / UI_TASK_PERIOD_MS)

/* 串口打印周期：500ms 输出一次，避免 9600 波特率下占用过多时间 */
#define UI_SERIAL_PERIOD_TICKS      (500 / UI_TASK_PERIOD_MS)

static ui_debug_page_t s_page = UI_DEBUG_INFO;
static u8  s_active = 0;        /* DEBUG 页是否激活：仅在 enter 后置 1，exit 后清 0 */
static u16 s_act_tick = 0;
static u16 s_serial_tick = 0;

/* 直线前进控制 */
static int s_fwd_speed = UI_MOTOR_FWD_SPEED_PCT;   /* 当前前进速度 */

/* 电机补偿标定状态 */
static u8  s_motor_calib_state = 0;     /* 0=空闲, 1=运行中, 2=完成 */
static u16 s_motor_calib_tick = 0;
static s32 s_calib_enc_left_start = 0;
static s32 s_calib_enc_right_start = 0;
static s32 s_calib_enc_left = 0;
static s32 s_calib_enc_right = 0;
static float s_calib_left_gain = 1.0f;
static float s_calib_right_gain = 1.0f;

static void ui_debug_stop_motor(void);

/* 清空 DEBUG 页面显示区域 */
static void ui_debug_clear(void)
{
    oled_spi_clear();
}

/* 绘制页面顶部标题 */
static void ui_debug_draw_header(void)
{
    char buf[16];
    const char *page_names[] = {"INFO", "MIC","TRACE","ACT","USONIC","MOTOR","ENCODER"};
    if (s_page < sizeof(page_names)/sizeof(page_names[0]))
    {
        sprintf(buf, "DBG:%s", page_names[s_page]);
    }
    else
    {
        sprintf(buf, "DBG:??");
    }
    oled_spi_show_string(0, 0, (u8 *)buf, 8);
}

/* 综合内部变量页 */
static void ui_debug_draw_info(void)
{
    char buf[32];
    int left = 0;
    int right = 0;
    int target_left = 0;
    int target_right = 0;
    s32 enc_left = 0;
    s32 enc_right = 0;

    {
        float dist = obstacle_guard_get_distance();
        int dist_display;
        if (dist >= 49.0f) {
            dist_display = -1;
        } else {
            dist_display = (int)dist;
        }
        sprintf(buf, "E:%+4d D:%3d",
                (int)(trace_control_get_error() * 100.0f),
                dist_display);
    }
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "V:%2d L:%d/%d",
            vehicle_get_speed_cm_s(),
            lap_counter_get_laps(), 2);
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    motion_get_target(&target_left, &target_right);
    sprintf(buf, "TL:%+4d TR:%+4d", target_left, target_right);
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    motion_get_current(&left, &right);
    sprintf(buf, "ML:%+4d MR:%+4d", left, right);
    oled_spi_show_string(0, 4, (u8 *)buf, 8);

    sprintf(buf, "FSM:%d", (int)fsm_get_state());
    oled_spi_show_string(0, 5, (u8 *)buf, 8);

    encoder_get_counts(&enc_left, &enc_right);
    sprintf(buf, "ENC:%+5ld/%+5ld", enc_left, enc_right);
    oled_spi_show_string(0, 6, (u8 *)buf, 8);
}

/* 循迹原始数据页 */
static void ui_debug_draw_trace(void)
{
    char buf[32];
    u16 adc[BSP_TRACE_CH_COUNT];
    u8 lost_cnt = 0;
    u8 recovery_cnt = 0;

    trace_read(adc);
    trace_control_get_lost_info(&lost_cnt, &recovery_cnt);

    sprintf(buf, "A0:%4d A1:%4d", adc[0], adc[1]);
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "A2:%4d A3:%4d", adc[2], adc[3]);
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    /* 显示误差和丢线计数 */
    sprintf(buf, "E:%+4d LE:%+4d",
            (int)(trace_control_get_error() * 100.0f),
            (int)(trace_control_get_last_error() * 100.0f));
    oled_spi_show_string(0, 3, (u8 *)buf, 8);
    sprintf(buf, "LC:%d RC:%d", lost_cnt, recovery_cnt);
    oled_spi_show_string(0, 4, (u8 *)buf, 8);
}

/* 系统状态页已暂时移除，简化调试界面
static void ui_debug_draw_state(void)
{
    char buf[32];
    s32 enc_left = 0;
    s32 enc_right = 0;
    u16 echo_us = 0;
    u8 us_ok = 0;
    u8 us_listen = 0;

    sprintf(buf, "FSM:%d MOT:%d",
            (int)fsm_get_state(), (int)motion_get_state());
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    sprintf(buf, "LOST:%d TIME:%d",
            trace_control_is_lost(), ui_get_run_seconds());
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    sprintf(buf, "MODE:%d D:%5lu",
            (int)ui_get_mode(), vehicle_get_distance_cm());
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    encoder_get_counts(&enc_left, &enc_right);
    sprintf(buf, "ENC:%+5ld/%+5ld", enc_left, enc_right);
    oled_spi_show_string(0, 4, (u8 *)buf, 8);

    ultrasonic_get_raw(&echo_us, &us_ok, &us_listen);
    sprintf(buf, "US:%4d O:%d I:%d", echo_us, us_ok, us_listen);
    oled_spi_show_string(0, 5, (u8 *)buf, 8);
}
*/

/* 外部报警 LED + 蜂鸣器自检页 */
static void ui_debug_draw_act(void)
{
    char buf[32];

    if (fsm_get_state() == FSM_STATE_IDLE ||
        fsm_get_state() == FSM_STATE_FINISHED)
    {
        sprintf(buf, "ACT:RUN T:%d", s_act_tick);
    }
    else
    {
        sprintf(buf, "ACT:SKIP S:%d", (int)fsm_get_state());
    }

    oled_spi_show_string(0, 1, (u8 *)buf, 8);
}

/* 运行 LED + 蜂鸣器自检逻辑 */
static void ui_debug_update_act(void)
{
    if (fsm_get_state() != FSM_STATE_IDLE &&
        fsm_get_state() != FSM_STATE_FINISHED)
    {
        led_alarm_off();
        return;
    }

    s_act_tick++;

    if ((s_act_tick % UI_ACT_LED_PERIOD_TICKS) == 0)
    {
        led_alarm_toggle();
    }

    if ((s_act_tick % UI_ACT_BEEP_PERIOD_TICKS) == 0)
    {
        buzzer_beep(1000, 100);
    }
}

/* 停止 LED + 蜂鸣器自检 */
static void ui_debug_stop_act(void)
{
    s_act_tick = 0;
    led_alarm_off();
    buzzer_off();
}

/* ===================== 声控 MIC 调试页 ===================== */
static u16 mic_last_adc = 0;           /* 上次 ADC 值 */
static u8  mic_trigger_count = 0;      /* 触发计数器（本次进入页面后） */
static u16 mic_last_trigger_tick = 0;  /* 上次触发时刻 */
static u8  mic_last_tap = 0;           /* 上次触发类型：1=单次，2=双次 */

/* 声控咪头检测页 - 显示 ADC 值、阈值判断、触发历史等 */
static void ui_debug_draw_mic(void)
{
    char buf[36];
    u16 adc = mic_get_raw();            /* 读取当前 ADC 值（上次转换结果） */
    u8 enabled = mic_is_enabled();      /* 读取使能状态 */
    u8 sound = (adc < MIC_ADC_THRESHOLD) ? 1 : 0;  /* ADC低于阈值=有声音 */

    /* 第1行：标题 + 使能状态 + 变化指示 */
    sprintf(buf, "MIC:%s  %s",
            enabled ? "ON" : "OFF",
            (adc != mic_last_adc) ? "*" : " ");
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    /* 第2行：ADC 值 + 阈值判断结果 */
    sprintf(buf, "ADC:%-5d%s", adc, sound ? "(HI/SND)" : "(LOW/SIL)");
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    /* 第3行：触发统计 */
    {
        u16 now = 0;
        u16 elapsed = 0;
        
        now = (u16)(app_get_tick() & 0xFFFF);  /* 取低16位即可 */
        if (now >= mic_last_trigger_tick)
        {
            elapsed = now - mic_last_trigger_tick;
        }
        else
        {
            elapsed = 0xFFFF - mic_last_trigger_tick + now;
        }

        sprintf(buf, "TRG:%d %c %dms ago",
                mic_trigger_count,
                (mic_last_tap == 2) ? 'D' : ((mic_last_tap == 1) ? 'S' : '-'),
                elapsed);
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
    }

    /* 第4行：消抖参数 + ADC 阈值 */
    sprintf(buf, "DEB:2 THR:%d", MIC_ADC_THRESHOLD);
    oled_spi_show_string(0, 4, (u8 *)buf, 8);

    /* 第5-6行：操作提示 */
    oled_spi_show_string(0, 5, (u8 *)"KEY1:TGL EN/DIS", 8);

    if (enabled)
    {
        oled_spi_show_string(0, 6, (u8 *)"1CLAP:ST 2:SP", 8);
    }
    else
    {
        oled_spi_show_string(0, 6, (u8 *)"MIC DISABLED", 8);
    }

    /* 更新内部状态（用于检测 ADC 值变化） */
    mic_last_adc = adc;

    /* 检测是否刚被触发（通过观察tick变化或调用mic_scan的返回值） */
    /* 注意：这里不主动调用mic_scan，避免重复扫描 */
}

/* 进入MIC调试页时的初始化 */
static void ui_debug_enter_mic(void)
{
    mic_last_adc = mic_get_raw();
    mic_trigger_count = 0;
    mic_last_tap = 0;
    mic_last_trigger_tick = (u16)(app_get_tick() & 0xFFFF);
}

/* 外部调用：当 app.c 检测到有效拍手触发时更新计数器 */
void ui_debug_mic_on_trigger(u8 tap)
{
    if (s_active && s_page == UI_DEBUG_MIC)
    {
        mic_trigger_count++;
        mic_last_tap = tap;
        mic_last_trigger_tick = (u16)(app_get_tick() & 0xFFFF);
    }
}

/* 电机自检页 */
static void ui_debug_draw_motor(void)
{
    char buf[32];

    if (ui_debug_motor_active())
    {
        sprintf(buf, "MOT:FWD %d%%", s_fwd_speed);
    }
    else
    {
        sprintf(buf, "MOT:IDLE S:%d", (int)fsm_get_state());
    }

    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    /* 显示标定结果或操作提示 */
    if (s_motor_calib_state == 2)
    {
        sprintf(buf, "L:%ld R:%ld", s_calib_enc_left, s_calib_enc_right);
        oled_spi_show_string(0, 3, (u8 *)buf, 8);
        sprintf(buf, "LG:%.2f RG:%.2f", s_calib_left_gain, s_calib_right_gain);
        oled_spi_show_string(0, 4, (u8 *)buf, 8);
        oled_spi_show_string(0, 5, (u8 *)"KEY:REDO", 8);
    }
    else
    {
        oled_spi_show_string(0, 3, (u8 *)"KEY UP/DN:SPD", 8);
        oled_spi_show_string(0, 4, (u8 *)"KEY EXT:CALIB", 8);
        oled_spi_show_string(0, 5, (u8 *)"HOLD 2s:EXIT", 8);
    }

    /* 显示当前增益 */
    {
        float left_g = 1.0f, right_g = 1.0f;
        motion_get_gains(&left_g, &right_g);
        sprintf(buf, "GAIN L%.2f R%.2f", left_g, right_g);
        oled_spi_show_string(0, 6, (u8 *)buf, 8);
    }
}

/* 启动电机补偿标定：临时禁用补偿，直行采集编码器脉冲 */
static void ui_debug_start_motor_calib(void)
{
    if (!ui_debug_motor_active())
    {
        return;
    }

    s_motor_calib_state = 1;
    s_motor_calib_tick = 0;
    s_calib_enc_left = 0;
    s_calib_enc_right = 0;

    /* 标定时禁用现有补偿，确保测得的是电机真实差异 */
    motion_set_gains(1.0f, 1.0f);

    /* 清零编码器起始值 */
    encoder_get_counts(&s_calib_enc_left_start, &s_calib_enc_right_start);

    /* 固定速度直行 */
    motion_set_speed(UI_MOTOR_CALIB_SPEED_PCT);
}

/* 结束电机补偿标定：计算并应用建议增益 */
static void ui_debug_finish_motor_calib(void)
{
    s32 left_now = 0, right_now = 0;
    float ratio;

    motion_stop();

    encoder_get_counts(&left_now, &right_now);
    s_calib_enc_left = left_now - s_calib_enc_left_start;
    s_calib_enc_right = right_now - s_calib_enc_right_start;

    /* 计算建议增益：降低较快一侧的输出 */
    s_calib_left_gain = 1.0f;
    s_calib_right_gain = 1.0f;

    if (s_calib_enc_left > 0 && s_calib_enc_right > 0)
    {
        if (s_calib_enc_left > s_calib_enc_right)
        {
            ratio = (float)s_calib_enc_right / (float)s_calib_enc_left;
            s_calib_left_gain = ratio;
        }
        else
        {
            ratio = (float)s_calib_enc_left / (float)s_calib_enc_right;
            s_calib_right_gain = ratio;
        }
    }

    /* 应用标定结果 */
    motion_set_gains(s_calib_left_gain, s_calib_right_gain);

    s_motor_calib_state = 2;
}

/* 运行电机直线前进逻辑 */
static void ui_debug_update_motor(void)
{
    if (!ui_debug_motor_active())
    {
        /* 非激活时不在这里停车，避免与 FSM 启动冲突；
         * 切页/退出时会通过 ui_debug_stop_all() 显式停车 */
        return;
    }

    /* 标定状态机优先 */
    if (s_motor_calib_state == 1)
    {
        s_motor_calib_tick++;
        if (s_motor_calib_tick >= UI_MOTOR_CALIB_TICKS)
        {
            ui_debug_finish_motor_calib();
        }
        return;
    }

    /* 直线前进 */
    motion_set_wheels(s_fwd_speed, s_fwd_speed);
}

/* 停止电机 */
static void ui_debug_stop_motor(void)
{
    s_fwd_speed = UI_MOTOR_FWD_SPEED_PCT;   /* 恢复默认速度 */
    s_motor_calib_state = 0;
    s_motor_calib_tick = 0;
    motion_stop();
}

/* 超声调试页：显示距离、原始回波、完成/监听标志、引脚电平、上下拉状态、超时状态 */
static void ui_debug_draw_ultrasonic(void)
{
    char buf[36];
    u16 echo_us = 0;
    u8 us_ok = 0;
    u8 us_listen = 0;
    u8 pin_high;

    ultrasonic_get_raw(&echo_us, &us_ok, &us_listen);
    pin_high = GPIO_ReadInputDataBit(BSP_US_ECHO_PORT, BSP_US_ECHO_PIN);

    /* 第1行：距离值（来自obstacle_guard缓存） */
    {
        float dist = obstacle_guard_get_distance();
        if (dist >= 49.0f) {
            sprintf(buf, "D: --cm");
        } else {
            sprintf(buf, "D:%3.0fcm", dist);
        }
    }
    oled_spi_show_string(0, 1, (u8 *)buf, 8);

    /* 第2行：回波时间 + 超时标记 */
    if (echo_us == 0xFFFF)
    {
        sprintf(buf, "ECHO:TOUT(999)");  /* 超时特殊标识 */
    }
    else
    {
        sprintf(buf, "ECHO:%4dus", echo_us);
    }
    oled_spi_show_string(0, 2, (u8 *)buf, 8);

    /* 第3行：测量状态 + 引脚电平 */
    sprintf(buf, "OK:%d LSTN:%d PIN:%c", us_ok, us_listen, pin_high ? 'H' : 'L');
    oled_spi_show_string(0, 3, (u8 *)buf, 8);

    /* 第4行：原始距离（直接读取） - 上下拉调节已禁用 */
    {
        float raw_dist = ultrasonic_get_distance();  /* EMA滤波后的值 */

        if (raw_dist >= 99.0f)
        {
            sprintf(buf, "RAW:>99cm");  /* 远方标识 */
        }
        else
        {
            sprintf(buf, "RAW:%.1fcm", raw_dist);
        }
    }
    oled_spi_show_string(0, 4, (u8 *)buf, 8);

    /* 第5行：超时诊断信息（新增） */
    {
        u32 timeout_cnt = ultrasonic_get_timeout_cnt();  /* 需要添加此接口 */
        sprintf(buf, "TOUT_CNT:%lu", (unsigned long)timeout_cnt);
    }
    oled_spi_show_string(0, 5, (u8 *)buf, 8);

    /* 第6行：操作提示 */
    oled_spi_show_string(0, 6, (u8 *)"KEY:NEXT PAGE", 8);
}

/* 编码器 PPR 测量页更新：KEY1=清零重测，KEY2=锁定当前值 */
static void ui_debug_update_encoder(void)
{
    if (key_start_scan())
    {
        /* KEY1: 清零计数器，开始新一轮测量 */
        encoder_reset_totals();
        s_enc_meas_locked[0] = 0;
        s_enc_meas_locked[1] = 0;
    }

    if (key_ext_scan())
    {
        /* KEY2: 锁定当前计数值为 PPR */
        s32 left = 0, right = 0;
        encoder_get_counts(&left, &right);

        if (left > 0 || right > 0)
        {
            s_enc_meas_locked[0] = left;
            s_enc_meas_locked[1] = right;

            /* 存入历史记录（循环覆盖） */
            if (s_enc_meas_cnt < ENC_MEASURE_MAX)
            {
                s_enc_meas[s_enc_meas_cnt][0] = left;
                s_enc_meas[s_enc_meas_cnt][1] = right;
                s_enc_meas_cnt++;
            }
            else
            {
                /* 满了就移位覆盖最早的一条 */
                u8 i;
                for (i = 0; i < ENC_MEASURE_MAX - 1; i++)
                {
                    s_enc_meas[i][0] = s_enc_meas[i + 1][0];
                    s_enc_meas[i][1] = s_enc_meas[i + 1][1];
                }
                s_enc_meas[ENC_MEASURE_MAX - 1][0] = left;
                s_enc_meas[ENC_MEASURE_MAX - 1][1] = right;
            }

            /* 锁存后自动清零准备下一次测量 */
            encoder_reset_totals();
        }
    }
}

/* 停止当前页可能正在运行的所有 IO 输出 */
static void ui_debug_stop_all(void)
{
    ui_debug_stop_act();
    ui_debug_stop_motor();
}

void ui_debug_enter(void)
{
    s_active = 1;
    s_page = UI_DEBUG_INFO;
    s_serial_tick = 0;
    ui_debug_stop_all();
    ui_debug_clear();

    /* 初始化各页面状态 */
    ui_debug_enter_mic();     /* 初始化MIC调试页状态 */
    ui_debug_enter_encoder(); /* 初始化编码器 PPR 测量页状态 */

    ui_debug_draw();
}

void ui_debug_exit(void)
{
    ui_debug_stop_all();
    s_active = 0;
    s_page = UI_DEBUG_INFO;
}

void ui_debug_next(void)
{
    ui_debug_stop_all();
    s_page = (ui_debug_page_t)((s_page + 1) % UI_DEBUG_MAX);
    ui_debug_clear();

    /* 重新初始化目标页面状态 */
    if (s_page == UI_DEBUG_ENCODER)
    {
        ui_debug_enter_encoder();
    }

    ui_debug_draw();
}

void ui_debug_draw(void)
{
    ui_debug_draw_header();

    switch (s_page)
    {
        case UI_DEBUG_INFO:
            ui_debug_draw_info();
            break;

        case UI_DEBUG_TRACE:
            ui_debug_draw_trace();
            break;

        /* UI_DEBUG_STATE 已暂时移除 */

        case UI_DEBUG_ACT:
            ui_debug_draw_act();
            break;

        case UI_DEBUG_MOTOR:
            ui_debug_draw_motor();
            break;

        case UI_DEBUG_ULTRASONIC:
            ui_debug_draw_ultrasonic();
            break;

        case UI_DEBUG_MIC:
            ui_debug_draw_mic();
            break;

        case UI_DEBUG_ENCODER:
            ui_debug_draw_encoder();
            break;

        default:
            break;
    }
}

/* 串口周期性输出关键内部变量，便于 PC 端查看/录波 */
static void ui_debug_serial_print(void)
{
    const char *name[] = {"INFO", "MIC", "TRACE", "ACT", "USONIC", "MOTOR", "ENCODER"};
    int target_left = 0, target_right = 0;
    int current_left = 0, current_right = 0;
    s32 enc_left = 0, enc_right = 0;
    u16 adc[BSP_TRACE_CH_COUNT];
    u8 lost_cnt = 0, recovery_cnt = 0;
    u16 echo_us = 0;
    u8 us_ok = 0, us_listen = 0;

    s_serial_tick++;
    if (s_serial_tick < UI_SERIAL_PERIOD_TICKS)
    {
        return;
    }
    s_serial_tick = 0;

    motion_get_target(&target_left, &target_right);
    motion_get_current(&current_left, &current_right);
    trace_read(adc);
    encoder_get_counts(&enc_left, &enc_right);
    trace_control_get_lost_info(&lost_cnt, &recovery_cnt);
    ultrasonic_get_raw(&echo_us, &us_ok, &us_listen);

    /* ── 编码器 PPR 校准专用输出 ──
     * 当处于 ENCODER 页面时，额外输出一行编码器专用数据，
     * 方便通过串口助手查看、录波、计算 PPR。
     *
     * 使用方法：
     *   1. 切换到 DEBUG → ENCODER 页面
     *   2. 手动缓慢转车轮一整圈
     *   3. 观察串口输出 "PPR_CAL: L=xxxx R=xxxx" 行
     *   4. 转一圈前后 L/R 的差值 = 单轮 PPR
     *   5. 多测几次取平均，填入 VEHICLE_ENCODER_PPR
     */
    if (s_page == UI_DEBUG_ENCODER)
    {
        printf("[ENC_CAL] L=%+6ld R=%+6ld  SPD=%2d cm/s  DIST=%4lu cm\r\n",
               enc_left, enc_right,
               vehicle_get_speed_cm_s(),
               (unsigned long)vehicle_get_distance_cm());
    }

    {
        float dist = obstacle_guard_get_distance();
        const char *dist_str;
        char dist_buf[8];
        if (dist >= 49.0f) {
            dist_str = "--";
        } else {
            sprintf(dist_buf, "%3d", (int)dist);
            dist_str = dist_buf;
        }

        printf("[DBG:%s] E:%+4d D:%s V:%2d L:%d "
               "A:%4d,%4d,%4d,%4d "
               "TL:%+4d TR:%+4d ML:%+4d MR:%+4d "
               "FSM:%d MOT:%d LOST:%d\r\n",
               name[s_page],
               (int)(trace_control_get_error() * 100.0f),
               dist_str,
               vehicle_get_speed_cm_s(),
               lap_counter_get_laps(),
               adc[0], adc[1], adc[2], adc[3],
               target_left, target_right,
               current_left, current_right,
               (int)fsm_get_state(),
               (int)motion_get_state(),
               trace_control_is_lost());
    }

    printf("[DBG:%s] EL:%+6ld ER:%+6ld LE:%+4d LC:%d RC:%d "
           "US:%4d O:%d I:%d\r\n",
           name[s_page],
           enc_left, enc_right,
           (int)(trace_control_get_last_error() * 100.0f),
           lost_cnt, recovery_cnt,
           echo_us, us_ok, us_listen);
}

void ui_debug_update(void)
{
    if (!s_active)
    {
        /* 安全开关：未进入 DEBUG 页时不应执行任何调试动作 */
        return;
    }

    if (s_page == UI_DEBUG_ACT)
    {
        ui_debug_update_act();
    }
    else if (s_page == UI_DEBUG_TRACE)
    {
        /* 阈值自学习已移除，TRACE 页面仅用于查看原始 ADC 与误差 */
        (void)0;
    }
    else if (s_page == UI_DEBUG_MOTOR)
    {
        /* MOTOR 页面下 KEY_EXT 短按触发/重新触发补偿标定 */
        if (key_ext_scan())
        {
            if (s_motor_calib_state == 0 || s_motor_calib_state == 2)
            {
                ui_debug_start_motor_calib();
            }
            else if (s_motor_calib_state == 1)
            {
                /* 运行中再次按下：提前结束并计算 */
                ui_debug_finish_motor_calib();
            }
        }

        /* 速度调节：KEY_START 短按切换速度档位 */
        if (key_start_scan() && s_motor_calib_state != 1)
        {
            s_fwd_speed += UI_MOTOR_FWD_SPEED_STEP;
            if (s_fwd_speed > UI_MOTOR_FWD_SPEED_MAX)
            {
                s_fwd_speed = UI_MOTOR_FWD_SPEED_MIN;   /* 循环回最小值 */
            }
        }

        ui_debug_update_motor();
    }
    else if (s_page == UI_DEBUG_MIC)
    {
        /* MIC 页面下 KEY1(短按) 切换声控使能/禁用 */
        if (key_start_scan())
        {
            u8 new_enabled = !mic_is_enabled();
            mic_set_enabled(new_enabled);

            /* 反馈提示 */
            if (new_enabled)
            {
                buzzer_beep(1200, 80);   /* 高音短鸣：已启用 */
            }
            else
            {
                buzzer_beep(800, 80);    /* 低音短鸣：已禁用 */
            }
        }

        /* 可选：KEY_EXT 长按重置触发计数器 */
        if (key_ext_scan())
        {
            mic_trigger_count = 0;
            mic_last_trigger_tick = (u16)(app_get_tick() & 0xFFFF);
        }
    }
    else if (s_page == UI_DEBUG_ENCODER)
    {
        ui_debug_update_encoder();
    }

    ui_debug_draw();
    ui_debug_serial_print();
}

ui_debug_page_t ui_debug_get_page(void)
{
    return s_page;
}

u8 ui_debug_motor_active(void)
{
    return (s_active &&
            s_page == UI_DEBUG_MOTOR &&
            fsm_get_state() == FSM_STATE_IDLE);
}

u8 ui_debug_is_active(void)
{
    return s_active;
}
