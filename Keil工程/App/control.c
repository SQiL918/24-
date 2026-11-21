#include "stm32f10x.h"
#include "control.h"
#include "app_config.h"
#include "bsp.h"
#include "imu.h"
#include "path.h"
#include "ui.h"
#include "gray_uart.h"

static float g_distance_mm = 0.0f;                  // 车身累计里程，单位 mm
static float g_speed_left = 0.0f;
static float g_speed_right = 0.0f;
static float g_speed_left_f = 0.0f;                 // 左轮速度低通滤波的状态，单位 mm/s
static float g_speed_right_f = 0.0f;                // 右轮速度低通滤波的状态，单位 mm/s
static float g_integ_left = 0.0f;                   // 左轮 PI 积分贡献，单位为 PWM 比较值
static float g_integ_right = 0.0f;                  // 右轮 PI 积分贡献，单位为 PWM 比较值
static uint8_t g_gray_bits = 0;

/**
  * 函    数：将浮点值限制在闭区间内
  * 参    数：v：待限幅值
  *           lo：下限
  *           hi：上限
  * 返 回 值：限幅后的值
  */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/**
  * 函    数：计算单轮轮速的前馈加 PI 输出
  * 参    数：target：目标轮速，单位 mm/s
  *           actual：滤波后的实测轮速，单位 mm/s
  *           integ：本轮积分状态的读写指针
  * 返 回 值：限幅后的 PWM 比较值，范围 0～PWM_MAX
  */
static float speed_pi(float target, float actual, float *integ)
{
    float err;
    float duty;

    if (target < 5.0f)                              // 接近零速目标时直接输出零
    {
        *integ = 0.0f;                              // 停轮时清积分，避免再次启动带入历史误差
        return 0.0f;
    }

    err = target - actual;                          // 目标减实测，正误差表示轮速偏低

    *integ += SPEED_KI * err * CONTROL_DT;          // 按控制周期累加误差的积分贡献
    *integ = clampf(*integ, -SPEED_I_LIMIT, SPEED_I_LIMIT);  // 限制积分幅度，减轻饱和后恢复时的过冲

    duty = SPEED_KFF * target + SPEED_KP * err + *integ;  // 目标速度前馈叠加比例和积分修正

    if (duty > 0.0f)
    {
        duty += SPEED_DEADBAND;                     // 正向输出叠加死区补偿，帮助克服起转阻力
    }

    return clampf(duty, 0.0f, (float)PWM_MAX);      // 映射到当前控制器允许的非负 PWM 范围
}

/**
  * 函    数：初始化轮速控制、里程和编码器，并制动电机
  * 参    数：无
  * 返 回 值：无
  */
void Control_Init(void)
{
    g_distance_mm = 0.0f;
    g_speed_left = 0.0f;
    g_speed_right = 0.0f;
    g_speed_left_f = 0.0f;
    g_speed_right_f = 0.0f;
    g_integ_left = 0.0f;
    g_integ_right = 0.0f;
    g_gray_bits = 0;

    BSP_EncoderReset();                             // 同步清零硬件编码器及差分基准
    BSP_MotorBrake();                               // 上电初始化完成后保持电机短路制动
}

/**
  * 函    数：开新一轮任务前清零里程、速度滤波和积分状态
  * 参    数：无
  * 返 回 值：无
  */
void Control_Reset(void)
{
    g_distance_mm = 0.0f;
    g_speed_left = 0.0f;
    g_speed_right = 0.0f;
    g_speed_left_f = 0.0f;
    g_speed_right_f = 0.0f;
    g_integ_left = 0.0f;
    g_integ_right = 0.0f;
    g_gray_bits = 0;

    BSP_EncoderReset();                             // 清硬件计数，保证新任务从零里程开始
    BSP_MotorBrake();                               // 重置期间制动两轮
}

/**
  * 函    数：执行一轮采样、路径决策和左右轮速度控制
  * 参    数：无
  * 返 回 值：无
  */
void Control_Step(void)
{
    int16_t d_left;
    int16_t d_right;
    float dist_delta;
    PathInput in;
    PathOutput out;

    BSP_Stage(BSP_STAGE_CONTROL);                   // 标记控制采样阶段，便于故障定位
    BSP_EncoderRead(&d_left, &d_right);             // 取最近一个采样周期的双轮编码器计数差

    /* 先解码串口缓冲中的灰度帧，让路径状态机使用最新可用的八路位图。 */
    Gray_Task();
    g_gray_bits = Gray_GetBits();                   // 保存控制周期内实际使用的位图，供路径和日志读取

    dist_delta = ((float)d_left + (float)d_right) * 0.5f / COUNTS_PER_MM;  // 双轮平均计数除以每毫米计数，得到车身增量
    g_distance_mm += dist_delta;                    // 累计本次运行里程，供路径切段

    g_speed_left  = (float)d_left  * CONTROL_HZ / COUNTS_PER_MM;  // 将左轮单周期计数换算为 mm/s
    g_speed_right = (float)d_right * CONTROL_HZ / COUNTS_PER_MM;  // 将右轮单周期计数换算为 mm/s

    /* 两轮分别做一阶低通，保留 60% 历史滤波值。 */
    g_speed_left_f  = 0.60f * g_speed_left_f  + 0.40f * g_speed_left;
    g_speed_right_f = 0.60f * g_speed_right_f + 0.40f * g_speed_right;

    BSP_Stage(BSP_STAGE_IMU);
    Imu_Update(CONTROL_DT);                         // 读取陀螺仪 Z 轴，在 5 ms 步长上更新航向

    BSP_Stage(BSP_STAGE_PATH);
    in.distance_mm = g_distance_mm;                 // 给路径状态机传递本周期里程
    in.yaw_deg = Imu_GetYaw();                      // 传递相对航向角，单位 °
    in.gyro_z_dps = Imu_GetGyroZ();                 // 传递滤波后的 Z 轴角速度，单位 °/s
    in.gray_bits = g_gray_bits;
    in.dt = CONTROL_DT;
    in.base_speed_mmps = (float)UI_GetSpeedSet();   // 读取按键选定的基础速度档位

    Path_Update(&in, &out);                         // 根据当前路径段生成双轮目标、制动和事件

    BSP_Stage(BSP_STAGE_OUTPUT);
    if (out.event != PATH_EVENT_NONE)
    {
        UI_Announce(out.event);                     // 将起跑、到点或结束事件交给声光提示
    }

    if (out.brake)
    {
        BSP_MotorBrake();                           // 路径要求停车时直接短路制动，并清除两轮积分
        g_integ_left = 0.0f;
        g_integ_right = 0.0f;
    }
    else
    {
        float duty_left = speed_pi(out.left_mmps, g_speed_left_f, &g_integ_left);  // 左轮速度闭环计算
        float duty_right = speed_pi(out.right_mmps, g_speed_right_f, &g_integ_right);  // 右轮速度闭环计算

        BSP_MotorSet((int16_t)duty_left, (int16_t)duty_right);  // 将两路 PWM 比较值交给 TIM2 电机驱动
    }
}

/**
  * 函    数：获取滤波后的左轮速度
  * 参    数：无
  * 返 回 值：左轮速度，单位 mm/s
  */
float Control_GetLeftSpeed(void)
{
    return g_speed_left_f;
}

/**
  * 函    数：获取滤波后的右轮速度
  * 参    数：无
  * 返 回 值：右轮速度，单位 mm/s
  */
float Control_GetRightSpeed(void)
{
    return g_speed_right_f;
}

/**
  * 函    数：获取自本次控制复位以来的累计里程
  * 参    数：无
  * 返 回 值：左右轮平均累计里程，单位 mm
  */
float Control_GetDistance(void)
{
    return g_distance_mm;
}

/**
  * 函    数：获取控制周期中使用的灰度位图
  * 参    数：无
  * 返 回 值：八路黑白位图，位值 1 表示黑
  */
uint8_t Control_GetGray(void)
{
    return g_gray_bits;
}

/**
  * 函    数：响应 TIM1 更新中断并驱动 5 ms 控制周期
  * 参    数：无
  * 返 回 值：无
  */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  // 先清更新标志，避免退出中断后立即重入

        BSP_Tick5ms();                              // 推进全局 5 ms 时基
        Control_Step();                             // 执行本周期运动控制
        UI_Tick5ms();                               // 推进声光提示的分段定时
    }
}
