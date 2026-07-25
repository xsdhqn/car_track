#include "app_fsm.h"
#include "../motion/motion_control.h"
#include "../vehicle/vehicle_state.h"
#include "../dashboard/ui_dashboard.h"
#include "../dashboard/ui_debug.h"
#include "../trace/trace_control.h"
#include "../obstacle/obstacle_guard.h"
#include "../mission/lap_counter.h"
#include "../alarm/alarm_control.h"
#include "../../BSP/input/key.h"
#include "../../APP/app.h"

/* A 点停车时长：规则模式下每圈到 A 点停车 10s */
#define A_STOP_MS               10000

/* 运行总时长保护：超过 99s 强制结束，防止小车冲出赛道后无限运行 */
#define RUN_TIMEOUT_MS          99000

/* 状态机私有变量 */
static fsm_state_t s_state = FSM_STATE_IDLE;
static u16 s_brake_ticks = 0;
static u32 s_run_ms = 0;

/* ===================== 内部状态切换 ===================== */

static void fsm_enter_running(void)
{
    s_state = FSM_STATE_RUNNING;
    /* 从 AVOIDING/BRAKING 恢复时，先释放刹车再进入运行 */
    motion_stop();
    obstacle_guard_clear_emergency_request();
    alarm_control_event(ALARM_EVENT_IDLE);
    alarm_control_set_stat_mode(LED_MODE_STEADY_ON);
}

static void fsm_enter_avoiding(void)
{
    s_state = FSM_STATE_AVOIDING;
    motion_brake();
    alarm_control_event(ALARM_EVENT_AVOIDING);
    alarm_control_set_stat_mode(LED_MODE_FAST_BLINK);
}

static void fsm_enter_braking(void)
{
    s_state = FSM_STATE_BRAKING;
    motion_brake();
    s_brake_ticks = (u16)(A_STOP_MS / APP_FSM_PERIOD_MS);
    alarm_control_event(ALARM_EVENT_BRAKING);
    alarm_control_set_stat_mode(LED_MODE_DOUBLE_FLASH);
}

static void fsm_enter_finished(void)
{
    s_state = FSM_STATE_FINISHED;
    motion_brake();     /* 到终点后能耗制动，避免惯性冲线 */
    ui_timer_stop();
    alarm_control_event(ALARM_EVENT_FINISHED);
    alarm_control_set_stat_mode(LED_MODE_BREATH);
}

/* 运行时间累加并检查是否超时；超时则自动切到 FINISHED */
static void fsm_update_run_timer(void)
{
    s_run_ms += APP_FSM_PERIOD_MS;
    if (s_run_ms >= RUN_TIMEOUT_MS)
    {
        fsm_enter_finished();
    }
}

static u8 fsm_is_timeout(void)
{
    return s_run_ms >= RUN_TIMEOUT_MS;
}

/* ===================== 接口实现 ===================== */

void fsm_init(void)
{
    s_state = FSM_STATE_IDLE;
    s_brake_ticks = 0;
    s_run_ms = 0;

    motion_init();
    trace_control_init();
    obstacle_guard_init();
    lap_counter_init();
    alarm_control_init();
    alarm_control_set_stat_mode(LED_MODE_SLOW_BLINK);
}

void fsm_start(void)
{
    if (s_state == FSM_STATE_IDLE || s_state == FSM_STATE_FINISHED)
    {
        fsm_reset();
        lap_counter_reset();
        fsm_enter_running();
        ui_timer_reset();
        ui_timer_start();
        alarm_control_event(ALARM_EVENT_START);
    }
}

void fsm_reset(void)
{
    s_state = FSM_STATE_IDLE;
    s_brake_ticks = 0;
    s_run_ms = 0;

    motion_stop();
    trace_control_init();
    obstacle_guard_init();
    lap_counter_reset();

    /* 清零车速/里程：确保每次启动都是从零开始统计 */
    vehicle_reset_distance();

    alarm_control_event(ALARM_EVENT_IDLE);
    alarm_control_set_stat_mode(LED_MODE_SLOW_BLINK);
    ui_timer_stop();
    ui_timer_reset();
}

fsm_state_t fsm_get_state(void)
{
    return s_state;
}

void fsm_update(void)
{
    int left_pct = 0;
    int right_pct = 0;

    /* 状态机管辖的业务模块更新（与 app.c 的多速率调度对齐） */
    alarm_control_update();

    switch (s_state)
    {
        case FSM_STATE_IDLE:
        {
            s_run_ms = 0;

            /* DEBUG 电机自检时由调试页控制运动，FSM 不强制停车 */
            if (!ui_debug_motor_active())
            {
                motion_stop();
            }

            /* 使用外接按键 KEY_EXT 作为启动键，避免与 KEY_START 的 UI 功能冲突。
             * 在 MOTOR 调试页时 KEY_EXT 被调试页占用做标定，FSM 不再响应。 */
            if (!ui_debug_motor_active() && key_ext_scan())
            {
                fsm_start();
            }
            break;
        }

        case FSM_STATE_RUNNING:
        {
            /* 计圈：两种模式均启用，正常模式自由循迹也计圈 */
            lap_counter_update();

            /* 运行总时长保护：超过 99s 强制结束 */
            fsm_update_run_timer();
            if (fsm_is_timeout())
            {
                break;
            }

            /* 处理黑点过线：每个黑点声光提示，第 4 个为 A 点计一圈 */
            if (lap_counter_just_passed_cross())
            {
                alarm_control_event(ALARM_EVENT_A_POINT);
            }

            if (lap_counter_just_passed_a())
            {
                if (lap_counter_is_finished())
                {
                    fsm_enter_finished();
                    break;
                }

                /* 规则模式：每圈到 A 点停车 10s */
                if (ui_get_mode() == UI_MODE_RULE)
                {
                    fsm_enter_braking();
                    break;
                }
            }

            /* 保险：若运动层仍处于刹车状态，先释放再下发目标 */
            if (motion_get_state() == MOTION_STATE_BRAKE)
            {
                motion_stop();
            }

            /* 执行循迹：获取目标轮速并下发 */
            trace_control_get_wheel_targets(&left_pct, &right_pct);

            /* 丢线寻线超时：先停车等待，不直接结束。
             * 若后续重新寻到线，FSM 仍在 RUNNING，会自动恢复运行。
             * 99s 总运行超时保护仍可作为最终安全兜底。 */
            if (trace_control_is_lost_timeout())
            {
                motion_stop();
                break;
            }

            motion_set_wheels(left_pct, right_pct);
            break;
        }

        case FSM_STATE_AVOIDING:
        {
            /* 运行总时长保护：超过 99s 强制结束 */
            fsm_update_run_timer();
            if (fsm_is_timeout())
            {
                break;
            }

            motion_brake();

            if (obstacle_guard_is_cleared())
            {
                alarm_control_event(ALARM_EVENT_IDLE);
                fsm_enter_running();
            }
            break;
        }

        case FSM_STATE_BRAKING:
        {
            /* 运行总时长保护：超过 99s 强制结束 */
            fsm_update_run_timer();
            if (fsm_is_timeout())
            {
                break;
            }

            motion_brake();

            if (s_brake_ticks > 0)
            {
                s_brake_ticks--;
            }
            else
            {
                alarm_control_event(ALARM_EVENT_IDLE);
                fsm_enter_running();
            }
            break;
        }

        case FSM_STATE_FINISHED:
        {
            /* 维持能耗制动，防止惯性冲线 */
            motion_brake();
            break;
        }

        default:
            break;
    }
}

/* 紧急刹车入口：由 app.c 的独立刹车块调用。
 * 此处直接执行刹车并同步状态与报警，不依赖调用方是否已先刹车。 */
void fsm_handle_emergency_brake(void)
{
    if (s_state == FSM_STATE_RUNNING)
    {
        motion_brake();                    /* 安全冗余：确保电机立即制动 */
        obstacle_guard_clear_emergency_request();
        fsm_enter_avoiding();
    }
}
