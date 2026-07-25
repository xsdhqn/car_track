#include "app.h"
#include "dashboard/ui_dashboard.h"
#include "dashboard/ui_debug.h"
#include "vehicle/vehicle_state.h"
#include "motion/motion_control.h"
#include "control/app_fsm.h"
#include "obstacle/obstacle_guard.h"
#include "trace/trace_control.h"
#include "../BSP/sensor/mic.h"
#include "../BSP/actuator/buzzer.h"

static u8 s_tick_div = 0;
static volatile u32 s_sys_tick = 0;

void app_init(void)
{
    vehicle_init();
    motion_init();
    fsm_init();
    ui_init();
}

u32 app_get_tick(void)
{
    return s_sys_tick;
}

/* TIM2 Update 中断：10kHz / 10 = 1kHz，作为系统节拍 */
void TIM2_IRQHandler(void)
{
    static u16 s_tim2_div = 0;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        if (++s_tim2_div >= 10)
        {
            s_tim2_div = 0;
            s_sys_tick++;
        }
    }
}

void app_update(void)
{
    s_tick_div++;
    if (s_tick_div >= (APP_VEHICLE_PERIOD_MS / APP_TICK_MS))
    {
        s_tick_div = 0;
    }

    /* ========== 最高优先级：独立紧急刹车块（10ms）==========
     * 超声波中断在测距完成瞬间设置紧急请求；
     * 这里不等 fsm_update() 的调度周期，立即刹车并同步状态机。
     */
    if (obstacle_guard_has_emergency_request() && fsm_get_state() == FSM_STATE_RUNNING)
    {
        motion_brake();                  /* 安全动作：先执行 */
        fsm_handle_emergency_brake();    /* 状态同步：改状态、清标志、报警 */
    }

    /* ========== 10ms 任务：避障检测 + 电机控制 + 声控检测 ========== */
    obstacle_guard_update();
    motion_update();

    /* 声控检测：DEBUG 模式任意拍手切换启停 */
    if (ui_debug_is_active())
    {
        u8 tap = mic_scan();
        if (tap != 0)
        {
            ui_debug_mic_on_trigger(tap);

            if (fsm_get_state() == FSM_STATE_IDLE || fsm_get_state() == FSM_STATE_FINISHED)
            {
                fsm_start();
            }
            else
            {
                motion_brake();
                fsm_reset();
            }
        }
    }

    /* ========== 20ms 任务：循迹控制 + 状态机 ========== */
    if (s_tick_div % (APP_TRACE_PERIOD_MS / APP_TICK_MS) == 0)
    {
        trace_control_update();
    }
    if (s_tick_div % (APP_FSM_PERIOD_MS / APP_TICK_MS) == 0)
    {
        fsm_update();
    }

    /* ========== 50ms 任务：UI ========== */
    if (s_tick_div % (APP_UI_PERIOD_MS / APP_TICK_MS) == 0)
    {
        ui_task();
    }

    /* ========== 100ms 任务：车速里程 + 刷新显示 ========== */
    if (s_tick_div % (APP_VEHICLE_PERIOD_MS / APP_TICK_MS) == 0)
    {
        vehicle_update();
        ui_set_speed(vehicle_get_speed_cm_s());
        ui_set_mileage(vehicle_get_distance_cm());
    }

    /* 驱动非阻塞蜂鸣器倒计时，放在所有可能发 beep 的模块之后 */
    buzzer_update();

    /* 精确等待下一个 10ms tick，替代 delay_ms 固定忙等。
     * 用 __WFI() 让 CPU 在 tick 到来前进入睡眠，节省功耗。 */
    {
        u32 target = s_sys_tick + APP_TICK_MS;
        while (s_sys_tick < target)
        {
            __WFI();
        }
    }
}
