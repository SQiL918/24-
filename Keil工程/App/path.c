#include "stm32f10x.h"
#include <math.h>
#include "path.h"
#include "app_config.h"
#include "imu.h"
#include "gray_uart.h"

#define PATH_MAX_SEGMENTS 16

/* 路径表的字段按顺序为段类型、长度或转角、速度倍率、弯向、到点提示。 */
static const Segment g_path_q1[] =
{
    { SEG_STRAIGHT, TRACK_AB_MM, 1.00f, 0, 1 },     // Q1 从 A 直行 TRACK_AB_MM，到 B 后结束
    { SEG_STOP,     0.0f,        0.00f, 0, 0 }
};

/* Q2 路线为 A→B→C→D→A，两段半圆均右转。
 * A→B 使用 SEG_LINE，灰度链路离线时回退到航向保持。
 * B→C、D→A 使用 SEG_ARC_LINE，编码器里程和 400 mm 弧径提供参考，灰度反馈纠偏。
 * B→C 收尾时若符合条件，先停车再绕内轮补角。C→D 使用 SEG_STRAIGHT 保持交接航向。
 * 改成 SEG_STRAIGHT 或 SEG_ARC 可分别使用航向保持直线和里程排程弧线。
 */
static const Segment g_path_q2[] =
{
    { SEG_LINE,     TRACK_AB_MM,          1.00f,  0, 1 },  // A→B，灰度循迹直线
    { SEG_ARC_LINE, TRACK_ARC_MM,         0.75f, -1, 1 },  // B→C，右半圆，灰度循迹
    { SEG_STRAIGHT, TRACK_CD_MM,          1.00f,  0, 1 },  // C→D，按交接航向直行
    { SEG_ARC_LINE, TRACK_ARC_MM,         0.75f, -1, 1 },  // D→A，右半圆，灰度循迹
    { SEG_STOP,     0.0f,                 0.00f,  0, 0 }
};

/* Q3 执行下表一圈，Q4 复用下表四圈。对角直线扣除转向时估算的前移距离。 */
static const Segment g_path_q3[] =
{
    { SEG_TURN,     +TRACK_ANGLE_DEG,     0.55f,  0, 0 },  // 在 A 左转，对准 AC
    { SEG_STRAIGHT, TRACK_DIAG_MM - TURN_FORWARD_MM, 1.00f, 0, 1 },  // 沿 AC 斜向直行到 C
    { SEG_TURN,     -TRACK_ANGLE_DEG,     0.55f,  0, 0 },  // 在 C 右转，对准进入 BC 弧线的方向
    { SEG_ARC,      TRACK_ARC_MM,         0.75f, -1, 1 },  // 走右半圆到 B，里程排程为主
    { SEG_TURN,     -TRACK_ANGLE_DEG,     0.55f,  0, 0 },  // 在 B 右转，对准 BD
    { SEG_STRAIGHT, TRACK_DIAG_MM - TURN_FORWARD_MM, 1.00f, 0, 1 },  // 沿 BD 斜向直行到 D
    { SEG_TURN,     +TRACK_ANGLE_DEG,     0.55f,  0, 0 },  // 在 D 左转，对准进入 DA 弧线的方向
    { SEG_ARC,      TRACK_ARC_MM,         0.75f, +1, 1 },  // 走左半圆回到 A，里程排程为主
    { SEG_STOP,     0.0f,                 0.00f,  0, 0 }
};

static PathState g_state = PATH_IDLE;
static uint8_t   g_mode = 0;
static uint8_t   g_seg_index = 0;
static uint8_t   g_lap = 0;                         // 当前圈索引，从 0 开始
static uint8_t   g_laps_total = 1;
static uint8_t   g_need_begin = 0;                  // 置位后由下一次控制周期初始化首段

static const Segment *g_path = 0;                   // 当前任务使用的分段路线表
static uint8_t g_path_len = 0;

static float g_seg_start_dist = 0.0f;               // 当前段开始时的累计里程，单位 mm
static float g_target_yaw = 0.0f;                   // 直线航向保持的目标角，单位 °
static float g_seg_yaw_start = 0.0f;                // 当前段起点的实际航向角，单位 °
static float g_arc_yaw_start = 0.0f;                // 弧段入口航向，用于按里程计算弧线参考
static float g_turn_target_yaw = 0.0f;              // 定角转向的绝对目标角，单位 °
static float g_last_line_err = 0.0f;                // 上次有效的归一化灰度误差，用来记住黑线侧
static float g_gray_corr = 0.0f;                    // 灰度产生的航向修正，单位 °
static float g_yaw_ref_dbg = 0.0f;                  // 最近计算的航向参考，供日志读取
static float g_nominal_yaw = 0.0f;                  // 可能交给下一直线段的目标航向
static float g_arc_radius = 0.0f;                   // 根据里程与转角估算的弧线半径，单位 mm
static float g_line_theta = 0.0f;                   // 灰度外环累计的航向偏置，单位 °
static float g_line_rate_dbg = 0.0f;                // 最新灰度角速度指令，单位 °/s
static float g_line_d_f = 0.0f;                     // 低通后的灰度误差变化率
static uint8_t g_arc_mode = 0;                      // 1 表示当前采用灰度循迹，0 表示其他模式
static uint8_t g_arc_why = 0;                       // 最近一次弧段结束原因编码
static ArcReport g_arc_rep;
static uint8_t  g_arc_rep_pending = 0;              // 置位表示有一份弧段报告等待读取
static int16_t  g_rep_e_min, g_rep_e_max;
static int16_t  g_rep_gc_min, g_rep_gc_max;
static uint16_t g_rep_lost;
static uint8_t  g_rep_g_min, g_rep_g_max;
static uint8_t g_nominal_yaw_valid = 0;
static uint8_t g_line_lost = 0;                     // 连续无方向信息的控制周期数
static uint16_t g_turn_ms = 0;
static uint16_t g_seg_ms = 0;
static uint8_t  g_settle_state = 0;                 // 出弯补角阶段，0 空闲、1 制动、2 转向
static uint16_t g_settle_ms = 0;
static uint8_t  g_settle_used = 0;                  // 本弧段已经触发过出弯补角
static float    g_track_yaw = 0.0f;                 // 以起跑方向为基准的赛道航向，按每个半圆累加 ±180°

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
  * 函    数：对黑色通道的索引求平均，估算黑线相对阵列中心的位置
  * 参    数：bits：八路黑白位图，位值 1 表示黑
  *           lost：写入检测结果的指针，0 有方向、1 无黑线、2 所有启用通道为黑
  * 返 回 值：归一化误差，OUT1 侧为负、OUT8 侧为正，无方向时沿用上次误差
  */
static float line_error(uint8_t bits, uint8_t *lost)
{
    int i;
    int count = 0;
    int weighted = 0;
    int active = 0;

    for (i = 0; i < 8; i++)
    {
        if (GRAY_MASK & (1u << i))
        {
            continue;                               // 跳过标记为故障的通道
        }
        active++;                                   // 只统计未屏蔽的有效通道
        if (bits & (1u << i))
        {
            weighted += i;                          // 累加压线通道的横向索引
            count++;
        }
    }

    /* 全白表示阵列看不到黑线，可根据上一次位置尝试回找。
     * 全黑常见于路口或过宽黑线，当前采样不提供左右方向信息。
     */
    if (count == 0)
    {
        *lost = 1;
        return g_last_line_err;
    }

    if (count >= active)
    {
        *lost = 2;
        return g_last_line_err;
    }

    *lost = 0;

    return (((float)weighted / (float)count) - 3.5f) / 3.5f;  // 以阵列中心索引 3.5 为零点，归一化至 ±1
}

/**
  * 函    数：以当前车况建立新段的里程和航向基准
  * 参    数：in：本周期采样输入，使用累计里程
  * 返 回 值：无
  */
static void begin_segment(const PathInput *in)
{
    const Segment *seg = &g_path[g_seg_index];

    g_seg_start_dist = in->distance_mm;             // 当前累计里程成为该段相对里程的零点
    g_seg_yaw_start = Imu_GetYaw();                 // 记录切段瞬间的实测航向
    g_arc_yaw_start = g_seg_yaw_start;
    /* 保留上一段最后一次有效的灰度偏差，供入段时看不到黑线的情况使用。 */
    g_gray_corr = 0.0f;
    g_line_theta = 0.0f;
    g_line_rate_dbg = 0.0f;
    g_line_d_f = 0.0f;
    g_arc_mode = 0;
    g_yaw_ref_dbg = Imu_GetYaw();
    g_arc_radius = 0.0f;
    /* 重新初始化本段弧线统计和计时状态。 */
    g_rep_e_min = 1000;  g_rep_e_max = -1000;
    g_rep_gc_min = 1000; g_rep_gc_max = -1000;
    g_rep_g_min = 255;   g_rep_g_max = 0;
    g_rep_lost = 0;
    g_line_lost = 0;
    g_turn_ms = 0;
    g_seg_ms = 0;
    g_settle_state = 0;
    g_settle_ms = 0;
    g_settle_used = 0;
    g_arc_why = 0;

    /* 如果上一弧段留下了可用的目标航向，紧接的 MPU 直线用它消化残差。
     * 其他段以当前实测航向开始。
     */
    if ((seg->type == SEG_STRAIGHT) && g_nominal_yaw_valid)
    {
        g_target_yaw = g_nominal_yaw;
    }
    else
    {
        g_target_yaw = Imu_GetYaw();
    }
    g_nominal_yaw_valid = 0;                        // 交接航向只允许传给紧接的下一段

    if (seg->type == SEG_TURN)
    {
        g_turn_target_yaw = Imu_GetYaw() + seg->length;  // 转向段的 length 表示相对转角，单位 °
    }
}

/**
  * 函    数：完成当前段，处理到点事件、换圈及路线结束
  * 参    数：in：本周期输入，用于初始化下一段
  *           out：写入过渡轮速、制动和事件的指针
  * 返 回 值：无
  */
static void advance_segment(const PathInput *in, PathOutput *out)
{
    const Segment *seg = &g_path[g_seg_index];

    if (seg->announce)
    {
        out->event = PATH_EVENT_POINT;              // 路径表要求到点提示时产生事件
    }

    g_seg_index++;                                  // 指向下一个路线表条目

    if (g_seg_index >= g_path_len)                  // 路线表越界时结束任务
    {
        g_state = PATH_DONE;
        out->brake = 1;
        out->event = PATH_EVENT_STOP;
        return;
    }

    if (g_path[g_seg_index].type == SEG_STOP)       // 路线表中的停车项同时标记一圈完成
    {
        if (g_lap + 1 < g_laps_total)
        {
            g_lap++;
            g_seg_index = 0;                        // 仍有剩余圈数时回到路线表首段
            begin_segment(in);
            out->left_mmps = in->base_speed_mmps;
            out->right_mmps = in->base_speed_mmps;
            return;
        }

        g_state = PATH_DONE;
        out->brake = 1;                             // 任务完成后要求电机短路制动
        out->left_mmps = 0.0f;
        out->right_mmps = 0.0f;
        out->event = PATH_EVENT_STOP;
        return;
    }

    begin_segment(in);

    /* 切段的这个 5 ms 周期先给两轮相同的基础速度，下一周期执行新段控制。 */
    out->left_mmps = in->base_speed_mmps;
    out->right_mmps = in->base_speed_mmps;
}

/* 灰度直线和灰度弧线共用同一转向控制。
 * 外环将灰度误差 e 和低通后的变化率变成角速度指令 w_line，再积分为航向偏置 theta。
 * 航向参考 = 路线切线方向 + theta，角速度参考 = 曲率角速度前馈 + w_line。
 * 内环按实测航向和陀螺仪角速度计算差速修正，再叠加曲率差速前馈。
 * 左轮目标为 base 减差速，右轮为 base 加差速。直线的曲率前馈为零。
 */
/**
  * 函    数：根据灰度偏差和陀螺仪反馈计算循迹轮速
  * 参    数：in：本周期航向、角速度和步长等输入
  *           out：写入左右轮目标速度的指针
  *           base：分段基础速度，单位 mm/s
  *           yaw_tangent：名义路线切线航向，单位 °
  *           turn_ff：差速前馈，单位 mm/s
  *           rate_ff：角速度前馈，单位 °/s
  *           err：当前归一化灰度误差
  *           prev_err：上一次归一化灰度误差
  *           lost：0 检出方向、1 全白、2 全黑
  *           reacq：非 0 时允许按上一侧回找黑线
  * 返 回 值：无
  */
static void line_steer(const PathInput *in, PathOutput *out, float base,
                       float yaw_tangent, float turn_ff, float rate_ff,
                       float err, float prev_err, uint8_t lost, uint8_t reacq)
{
    float w_line = 0.0f;
    float yaw_ref;
    float rate_ref;
    float turn_corr;

    if (lost == 1)
    {
        if (g_line_lost < 250) { g_line_lost++; }   // 累计连续全白周期并饱和于 250
        /* 丢线时若允许回找，先把航向偏置设到黑线上一次出现的方向。 */
        if (reacq)
        {
            if (g_last_line_err > 0.02f)
            {
                g_line_theta = -ARC_LINE_REACQ_DEG;  // 黑线在右侧，设为负航向偏置
            }
            else if (g_last_line_err < -0.02f)
            {
                g_line_theta = ARC_LINE_REACQ_DEG;  // 黑线在左侧，设为正航向偏置
            }
            else
            {
                g_line_theta *= ARC_GRAY_LOST_DECAY;  // 无明确的历史方向，逐步衰减旧偏置
            }
        }
        else
        {
            g_line_theta *= ARC_GRAY_LOST_DECAY;    // 本段后部不主动回找时逐步撤去偏置
        }
    }
    else if (lost == 2)
    {
        if (g_line_lost < 250) { g_line_lost++; }
        /* 全黑不提供左右方向，保持当前偏置。 */
    }
    else
    {
        float de = (err - prev_err) / in->dt;       // 用相邻控制周期的误差差分估计变化率

        /* 八路二值位图的误差会跳变，先低通再用于微分反馈。 */
        g_line_d_f = (1.0f - ARC_LINE_D_FILT) * g_line_d_f +
                     ARC_LINE_D_FILT * de;

        w_line = -(ARC_LINE_KP * err + ARC_LINE_KD * g_line_d_f);  // 误差为正时黑线偏右，给负角速度指令
        w_line = clampf(w_line, -ARC_LINE_W_MAX, ARC_LINE_W_MAX);  // 限制灰度指令的最大角速度

        g_line_theta += w_line * in->dt;            // 将角速度修正积分成航向偏置
        g_line_theta = clampf(g_line_theta,
                              -ARC_LINE_THETA_MAX, ARC_LINE_THETA_MAX);

        g_line_lost = 0;                            // 找回有效方向后清除连续丢线计数
    }

    g_line_rate_dbg = w_line;
    g_gray_corr = g_line_theta;                     // 共用调试字段，保存当前灰度航向偏置
    yaw_ref = yaw_tangent + g_line_theta;           // 切线航向叠加灰度偏置
    rate_ref = rate_ff + w_line;                    // 曲率角速度叠加灰度给出的修正
    g_yaw_ref_dbg = yaw_ref;

    if (Imu_IsReady())
    {
        /* 航向内环对角速度参考与实测角速度的差值做反馈。
         * 稳定过弯时目标角速度本就非零，因此保留 rate_ref。
         */
        turn_corr = ARC_YAW_KP * (yaw_ref - in->yaw_deg) -
                    ARC_YAW_KD * (in->gyro_z_dps - rate_ref);
    }
    else
    {
        /* IMU 未就绪时，使用灰度误差直接产生左右差速修正。 */
        turn_corr = -(ARC_LINE_FALLBACK_KP * clampf(err, -1.0f, 1.0f));
    }
    turn_corr = clampf(turn_corr, -ARC_CORR_LIMIT, ARC_CORR_LIMIT);  // 限制闭环差速修正，几何前馈随后叠加

    out->left_mmps  = base - (turn_ff + turn_corr);  // 差速为正时减小左轮目标，产生左转
    out->right_mmps = base + (turn_ff + turn_corr);  // 同一差速加到右轮目标
}

/* 航向保持直线，也用作灰度直线在链路离线时的回退控制。 */
/**
  * 函    数：通过航向误差与角速度阻尼保持直线方向
  * 参    数：in：本周期航向与角速度输入
  *           out：写入两轮目标速度的指针
  *           base：基础速度，单位 mm/s
  * 返 回 值：无
  */
static void straight_steer(const PathInput *in, PathOutput *out, float base)
{
    float err = g_target_yaw - in->yaw_deg;         // 直线目标航向减当前实测航向
    float turn = YAW_KP * err - YAW_KD * in->gyro_z_dps;  // 比例纠偏叠加角速度阻尼

    turn = clampf(turn, -YAW_TURN_LIMIT, YAW_TURN_LIMIT);  // 限制直线能施加的左右差速

    out->left_mmps = base - turn;
    out->right_mmps = base + turn;
}

/* 弧段结束时可校正软件航向，或将目标航向交给下一直线段。
 * 灰度弧目标由 ARC_SETTLE_ABS_YAW 决定，默认基于起跑方向逐弧累加。
 * 排程弧以入弧航向加上计划转角作为目标。
 */
/**
  * 函    数：收尾弧段、生成弧段诊断报告并切换路线
  * 参    数：in：本周期采样输入
  *           out：写入下段指令及事件的指针
  *           dist：本弧段累计行驶距离，单位 mm
  *           target：本弧段结束目标航向，单位 °
  * 返 回 值：无
  */
static void arc_finish(const PathInput *in, PathOutput *out, float dist,
                       float target)
{
    float resid = fabsf(target - in->yaw_deg);      // 计算结束时实测航向和目标航向的绝对差

    /* 只有残差很小时才修正软件航向；其余允许交接的残差由下一直线段逐步消化。 */
    if (resid <= ARC_YAW_SNAP_DEG)
    {
        Imu_SetYaw(target);
    }                                               // 对接近目标的实测航向做软件校正
    else if ((resid <= ARC_YAW_FIX_MAX) || (g_arc_mode && (resid <= 45.0f)))
    {
        g_nominal_yaw = target;
        g_nominal_yaw_valid = 1;
    }                                               // 标记下一紧接的 MPU 直线可采用交接航向

    if (g_arc_mode)
    {
        g_track_yaw = target;                       // 更新下一段灰度弧的名义航向基准
    }

    g_gray_corr = 0.0f;
    g_line_theta = 0.0f;
    g_line_rate_dbg = 0.0f;
    g_line_d_f = 0.0f;

    /* 发布本弧段的里程、航向、半径及循迹统计。 */
    if (g_rep_e_min > 999)  { g_rep_e_min = 0; }
    if (g_rep_e_max < -999) { g_rep_e_max = 0; }
    g_arc_rep.dist_mm   = (int16_t)dist;
    g_arc_rep.yaw_deg   = (int16_t)in->yaw_deg;
    g_arc_rep.radius_mm = (int16_t)g_arc_radius;
    g_arc_rep.e_min     = g_rep_e_min;
    g_arc_rep.e_max     = g_rep_e_max;
    g_arc_rep.gc_min    = (int8_t)g_rep_gc_min;
    g_arc_rep.gc_max    = (int8_t)g_rep_gc_max;
    g_arc_rep.g_min     = g_rep_g_min;
    g_arc_rep.g_max     = g_rep_g_max;
    g_arc_rep.lost      = g_rep_lost;
    g_arc_rep.mode      = g_arc_mode;
    g_arc_rep.why       = g_arc_why;
    g_arc_rep_pending   = 1;                        // 通知前台有一份弧段报告可读

    advance_segment(in, out);
}

/* 只有弧线后面紧接直线时才考虑出弯补角。 */
/**
  * 函    数：判断路径表中下一段是否为灰度或航向保持直线
  * 参    数：无
  * 返 回 值：1 表示下一段为直线，0 表示无下一段或为其他段
  */
static uint8_t Path_NextIsStraight(void)
{
    uint8_t next = (uint8_t)(g_seg_index + 1u);

    if (next >= g_path_len)
    {
        return 0;
    }
    if ((g_path[next].type == SEG_STRAIGHT) || (g_path[next].type == SEG_LINE))
    {
        return 1;
    }
    return 0;
}

/* 出弯交接先制动，再由外侧车轮驱动车身绕停住的内轮补角。
 * 达到目标容差或等待超时后，交回弧段收尾逻辑。
 */
/**
  * 函    数：执行弧线到直线的制动和定角补偿
  * 参    数：in：本周期航向和控制步长
  *           out：写入两轮目标速度的指针
  *           target_yaw：计划交接航向，单位 °
  * 返 回 值：0 表示制动或转向中，1 表示到位或转向超时
  */
static uint8_t arc_settle(const PathInput *in, PathOutput *out, float target_yaw)
{
    float err = target_yaw - in->yaw_deg;
    float v;

    if (g_settle_state == 0u)
    {
        g_settle_state = 1u;                        // 首次进入时从制动阶段开始
        g_settle_ms = 0;
    }

    g_settle_ms = (uint16_t)(g_settle_ms + (uint16_t)(in->dt * 1000.0f));  // 按控制周期步长累计阶段毫秒数

    if (g_settle_state == 1u)
    {
        /* 左右目标轮速置零，由速度环和板级输出将两轮制动。 */
        out->left_mmps = 0.0f;
        out->right_mmps = 0.0f;
        if (g_settle_ms >= ARC_SETTLE_BRAKE_MS)     // 等待制动阶段的固定时间后切换到转向
        {
            g_settle_state = 2u;
            g_settle_ms = 0;
        }
        return 0;
    }

    if (fabsf(err) <= ARC_SETTLE_DONE_TOL)          // 残差进入补角到位容差
    {
        Imu_SetYaw(target_yaw);                     // 将软件航向校正到目标值
        g_settle_state = 0u;
        g_settle_ms = 0;
        return 1;
    }

    if (g_settle_ms > TURN_TIMEOUT_MS)
    {
        /* 转向超时后结束补角，保留实测航向。
         * 后续由 arc_finish 判断残差是否可以交给直线段。
         */
        g_settle_state = 0u;
        g_settle_ms = 0;
        return 1;
    }

    v = TURN_KP * fabsf(err);                       // 按角度误差计算外轮目标速度
    v = clampf(v, TURN_MIN_MMPS, TURN_MAX_MMPS);    // 限速并保留起转所需的最低速度

    if (err > 0.0f)
    {
        /* 补角向左时停左轮，由右轮推进。 */
        out->left_mmps = 0.0f;
        out->right_mmps = v;
    }
    else
    {
        /* 补角向右时停右轮，由左轮推进。 */
        out->left_mmps = v;
        out->right_mmps = 0.0f;
    }
    return 0;
}

/* 一次弧段控制更新，通常由 5 ms 控制循环调用。
 * use_line 为 0 时，按编码器里程推进 180° 航向参考，有灰度误差信息时用它微调。
 * use_line 为 1 时，灰度误差经外环积成限幅的航向偏置，再叠加到名义切线角。
 */
/**
  * 函    数：执行一轮曲率前馈、弧线纠偏以及弧段收尾判定
  * 参    数：in：本周期路径输入
  *           out：写入轮速、事件和制动命令的指针
  *           seg：当前弧段定义
  *           base：当前弧段基础速度，单位 mm/s
  *           use_line：1 启用灰度循迹，0 选择里程排程弧线
  * 返 回 值：无
  */
static void arc_step(const PathInput *in, PathOutput *out, const Segment *seg,
                     float base, uint8_t use_line)
{
    float dist = in->distance_mm - g_seg_start_dist;  // 将累计里程换成从弧段起点算起的里程
    float progress;
    float yaw_sched;
    float err;
    float prev_err;
    uint8_t lost;
    float turn_ff;
    float rate_ff;
    float yaw_done;
    float line_done;
    float dist_done;

    g_arc_mode = use_line;                          // 保存本周期选择的弧线模式供诊断

    /* 将 180° 均匀分配到整段编码器里程，避免直接按时间推算弧线进度。
     * 灰度循迹模式把进度限制在 100%，排程模式允许稍许超出。
     */
    progress = dist / seg->length;
    progress = clampf(progress, 0.0f, use_line ? 1.00f : 1.05f);
    yaw_sched = g_arc_yaw_start + (float)seg->arc_dir * 180.0f * progress;  // 入口角加上按里程累计的带符号转角

    /* 从位图提取循迹误差，正值表示黑线靠 OUT8 一侧。 */
    prev_err = g_last_line_err;
    err = line_error(in->gray_bits, &lost);

    /* 弧段通过转角保护、临近终点时无方向信息、或编码器里程到限三种条件之一进入收尾。
     * 灰度弧后接直线时，符合条件会先制动补角。
     */
    /* 实测转角超过保护角度且已走过 97% 以上时可结束。 */
    yaw_done = (fabsf(in->yaw_deg - g_arc_yaw_start) > ARC_YAW_DONE_DEG) &&
               (dist > seg->length * 0.97f);
    /* 已走过 95% 以上且持续丢失方向时，还要检查入口航向加计划转角与实测值的差。 */
    line_done = (dist > seg->length * 0.95f) &&
                (g_line_lost >= ARC_LINE_LOST_SAMPLES) &&
                (fabsf((g_arc_yaw_start + (float)seg->arc_dir * 180.0f) -
                       in->yaw_deg) <= 10.0f);
    dist_done = (dist > seg->length + ARC_FINISH_MARGIN_MM);  // 允许比名义弧长多行驶配置的收尾余量

    if (yaw_done || line_done || dist_done)
    {
        /* 计算用于补角和交接的弧段结束目标。灰度弧默认从起跑方向开始逐弧累加角度。 */
        float target;
        float resid;

        if (use_line)
        {
#if ARC_SETTLE_ABS_YAW
            /* 以起跑时建立并逐弧推进的赛道航向为基准。 */
            target = g_track_yaw + (float)seg->arc_dir * 180.0f;
#else
            /* 以本弧段的入口航向为基准。 */
            target = g_arc_yaw_start + (float)seg->arc_dir * 180.0f;
#endif
        }
        else
        {
            target = g_arc_yaw_start + (float)seg->arc_dir * 180.0f;
        }
        resid = fabsf(target - in->yaw_deg);

        /* 下段是直线且当前启用了灰度和 IMU 时，超过容差便进入补角。
         * 已经进入补角时继续执行，直至到位或超时。
         */
        if (use_line && Imu_IsReady() && Path_NextIsStraight() &&
            ((resid > ARC_SETTLE_TOL) || (g_settle_used != 0u)))
        {
            g_settle_used = 1u;
            if (!arc_settle(in, out, target))
            {
                return;                             // 正在制动或转向，暂时保留当前弧段
            }
            g_arc_why = 4u;                         // 4 表示经过补角流程结束，可能到位或超时
        }
        else if (g_settle_used == 0u)
        {
            /* 结束原因：1 转角保护，2 临近终点无方向信息，3 里程到限。 */
            g_arc_why = yaw_done ? 1u : (line_done ? 2u : 3u);
        }

        arc_finish(in, out, dist, target);
        return;
    }

    /* 由轮距、基础速度和半径计算单侧的差速前馈量。 */
    turn_ff = (float)seg->arc_dir * base * CAR_TRACK_WIDTH_MM * ARC_FF_TRIM /
              (2.0f * TRACK_RADIUS_MM);
    rate_ff = (float)seg->arc_dir * 180.0f * base / seg->length;  // 计算与基础速度对应的名义角速度

    if (use_line)
    {
        line_steer(in, out, base, yaw_sched, turn_ff, rate_ff,
                   err, prev_err, lost, (dist < seg->length * 0.95f));
    }
    else
    {
        /* 里程排程弧线按名义航向行驶，灰度误差用于修正参考角。
         * 灰度驱动离线时仍保留最后一帧，当前分支可能收到旧位图。
         */
        float yaw_ref;
        float yaw_err;
        float turn_corr;

        if (lost == 1)
        {
            if (g_line_lost < 250) { g_line_lost++; }
            if (dist < seg->length * 0.95f)         // 临近段尾前允许朝上一次看到黑线的侧向纠偏
            {
                if (g_last_line_err > 0.02f)
                {
                    g_gray_corr = -ARC_REACQ_DEG;
                }
                else if (g_last_line_err < -0.02f)
                {
                    g_gray_corr = ARC_REACQ_DEG;
                }
                else
                {
                    g_gray_corr *= ARC_GRAY_LOST_DECAY;
                }
            }
            else
            {
                g_gray_corr *= ARC_GRAY_LOST_DECAY;  // 临近弧末，全白时只逐步衰减灰度偏置
            }
        }
        else if (lost == 2)
        {
            if (g_line_lost < 250) { g_line_lost++; }
            g_gray_corr *= ARC_GRAY_LOST_DECAY;     // 全黑缺乏方向信息，逐步减弱灰度偏置
        }
        else
        {
            float corr = -(ARC_GRAY_TRIM_KP * err +
                           ARC_GRAY_TRIM_KD * (err - prev_err));
            g_gray_corr = clampf(corr, -ARC_GRAY_CORR_MAX, ARC_GRAY_CORR_MAX);  // 限制灰度微调角，防止偏离排程目标太多
            g_line_lost = 0;
        }

        yaw_ref = yaw_sched + g_gray_corr;          // 将灰度微调角叠加到里程排程航向
        g_yaw_ref_dbg = yaw_ref;

        if (Imu_IsReady())
        {
            yaw_err = yaw_ref - in->yaw_deg;
            turn_corr = ARC_YAW_KP * yaw_err -
                        ARC_YAW_KD * (in->gyro_z_dps - rate_ff);
        }
        else
        {
            turn_corr = -(ARC_LINE_FALLBACK_KP * clampf(err, -1.0f, 1.0f));
        }
        turn_corr = clampf(turn_corr, -ARC_CORR_LIMIT, ARC_CORR_LIMIT);  // 约束闭环部分，几何前馈随后叠加

        out->left_mmps  = base - (turn_ff + turn_corr);
        out->right_mmps = base + (turn_ff + turn_corr);
    }

    g_last_line_err = err;                          // 记录本次误差，供下一控制周期使用

    /* 汇总本弧段的灰度误差、航向修正、位图范围和无方向周期数。 */
    {
        int16_t ev = (int16_t)(err * 100.0f);
        int16_t gv = (int16_t)g_gray_corr;

        if (ev < g_rep_e_min)  { g_rep_e_min = ev; }
        if (ev > g_rep_e_max)  { g_rep_e_max = ev; }
        if (gv < g_rep_gc_min) { g_rep_gc_min = gv; }
        if (gv > g_rep_gc_max) { g_rep_gc_max = gv; }
        if (in->gray_bits < g_rep_g_min) { g_rep_g_min = in->gray_bits; }
        if (in->gray_bits > g_rep_g_max) { g_rep_g_max = in->gray_bits; }
        if (lost) { g_rep_lost++; }
    }

    /* 按弧长除以扫过角度（弧度）估算转弯半径，供曲率前馈标定。
     * 只在转过约 60° 且仍处于弧段里程范围内时更新。
     */
    {
        float swept = fabsf(in->yaw_deg - g_arc_yaw_start) * 0.017453293f;
        if ((swept > 1.0472f) &&
            (dist <= seg->length + ARC_FINISH_MARGIN_MM))
        {
            g_arc_radius = dist / swept;
        }
    }
}

/**
  * 函    数：初始化路径状态机和循迹状态
  * 参    数：无
  * 返 回 值：无
  */
void Path_Init(void)
{
    g_state = PATH_IDLE;                            // 上电后处于待机状态
    g_mode = 0;
    g_seg_index = 0;
    g_lap = 0;
    g_laps_total = 1;
    g_need_begin = 0;
    g_path = 0;
    g_path_len = 0;
    g_last_line_err = 0.0f;
    g_gray_corr = 0.0f;
    g_yaw_ref_dbg = 0.0f;
    g_nominal_yaw = 0.0f;
    g_nominal_yaw_valid = 0;
    g_line_lost = 0;
    g_line_theta = 0.0f;
    g_line_rate_dbg = 0.0f;
    g_line_d_f = 0.0f;
    g_arc_mode = 0;
    g_seg_yaw_start = 0.0f;
    g_settle_state = 0;
    g_settle_ms = 0;
}

/**
  * 函    数：选择 Q1～Q4 路径并启动任务
  * 参    数：mode：模式编号，允许范围为 1～4，超出时钳位
  * 返 回 值：无
  */
void Path_Start(uint8_t mode)
{
    if (mode < 1) { mode = 1; }
    if (mode > 4) { mode = 4; }

    g_mode = mode;
    g_seg_index = 0;
    g_lap = 0;
    g_state = PATH_RUNNING;
    g_need_begin = 1;                               // 首段起点信息在下一控制周期登记
    /* 以进入任务时的航向为起跑基准。UI 在这里之前会重置航向。
     * 灰度弧结束后按目标转角推进赛道航向。
     */
    g_track_yaw = Imu_GetYaw();
    g_settle_state = 0;
    g_settle_ms = 0;
    g_settle_used = 0;

    switch (mode)
    {
        case 1:
            g_path = g_path_q1;
            g_path_len = (uint8_t)(sizeof(g_path_q1) / sizeof(g_path_q1[0]));
            g_laps_total = 1;                       // Q1 只执行一次
            break;
        case 2:
            g_path = g_path_q2;
            g_path_len = (uint8_t)(sizeof(g_path_q2) / sizeof(g_path_q2[0]));
            g_laps_total = 1;                       // Q2 只执行一圈
            break;
        case 3:
            g_path = g_path_q3;
            g_path_len = (uint8_t)(sizeof(g_path_q3) / sizeof(g_path_q3[0]));
            g_laps_total = 1;                       // Q3 只执行一圈
            break;
        case 4:
        default:
            g_path = g_path_q3;                     // Q4 使用和 Q3 相同的路线表
            g_path_len = (uint8_t)(sizeof(g_path_q3) / sizeof(g_path_q3[0]));
            g_laps_total = 4;                       // Q4 连续执行四圈
            break;
    }
}

/**
  * 函    数：将路径状态切回待机并清除待初始化标志
  * 参    数：无
  * 返 回 值：无，调用方可同时下达电机制动
  */
void Path_Stop(void)
{
    g_state = PATH_IDLE;
    g_need_begin = 0;
}

/**
  * 函    数：查询路径状态机是否仍在执行任务
  * 参    数：无
  * 返 回 值：1 为运行，0 为待机或已结束
  */
uint8_t Path_IsRunning(void)
{
    return (g_state == PATH_RUNNING) ? 1 : 0;
}

/**
  * 函    数：获取最近启动的任务编号
  * 参    数：无
  * 返 回 值：1～4 对应 Q1～Q4，初始化后尚未启动时为 0
  */
uint8_t Path_GetMode(void)
{
    return g_mode;
}

/**
  * 函    数：获取当前路线表位置
  * 参    数：无
  * 返 回 值：从 0 开始的分段索引
  */
uint8_t Path_GetSegment(void)
{
    return g_seg_index;
}

/**
  * 函    数：根据传感器输入执行一步路径状态机
  * 参    数：in：累计里程、航向、灰度位图、周期和基础速度
  *           out：写入左右轮目标、制动命令和路径事件的指针
  * 返 回 值：无
  */
void Path_Update(const PathInput *in, PathOutput *out)
{
    const Segment *seg;
    float base;
    float scale;

    /* 先初始化本周期输出，再按状态填入实际命令。 */
    out->left_mmps = 0.0f;
    out->right_mmps = 0.0f;
    out->brake = 0;
    out->event = PATH_EVENT_NONE;

    if (g_state != PATH_RUNNING)
    {
        /* 停车时仍更新灰度误差供串口读取，可在静止状态检查传感器的左右顺序。 */
        {
            uint8_t line_lost;
            g_last_line_err = line_error(in->gray_bits, &line_lost);
        }
        out->brake = 1;                             // 无活动任务时保持制动
        return;
    }

    if (g_need_begin)
    {
        g_need_begin = 0;
        begin_segment(in);                          // 记录首段里程、航向等初始条件
        out->event = PATH_EVENT_START;              // 首个运行周期产生起跑事件
    }

    g_seg_ms = (uint16_t)(g_seg_ms + (uint16_t)(in->dt * 1000.0f));  // 更新当前段已经占用的时间，单位 ms
    if (g_seg_ms > SEGMENT_TIMEOUT_MS)              // 任一段超时都结束整个任务
    {
        g_state = PATH_DONE;
        out->brake = 1;
        out->event = PATH_EVENT_STOP;
        return;
    }

    if (g_seg_index >= g_path_len)
    {
        g_state = PATH_DONE;
        out->brake = 1;
        out->event = PATH_EVENT_STOP;
        return;
    }

    seg = &g_path[g_seg_index];
    scale = (g_mode == 4) ? 1.15f : 1.0f;           // Q4 基础速度额外乘以 1.15
    /* 路线按段配置速度倍率，弧线使用 0.75。
     * 最终分段基础速度再按控制范围限幅。
     */
    base = clampf(in->base_speed_mmps * scale * seg->speed_ratio,
                  120.0f, MAX_SPEED_MMPS);

    switch (seg->type)
    {
        case SEG_STRAIGHT:
        {
            float dist = in->distance_mm - g_seg_start_dist;

            /* MPU 直线只用陀螺仪转向，同时记录可用的灰度误差方向，供下段入弯回找。 */
            {
                uint8_t line_lost;
                g_last_line_err = line_error(in->gray_bits, &line_lost);
            }

            if (dist >= seg->length)                // 直线走够本段里程后切换
            {
                advance_segment(in, out);
                return;
            }

            straight_steer(in, out, base);          // 用陀螺仪保持本段目标航向
            break;
        }

        case SEG_LINE:
        {
            float dist = in->distance_mm - g_seg_start_dist;
            uint8_t lost;
            uint8_t online;
            float err;

            if (dist >= seg->length)
            {
                advance_segment(in, out);
                return;
            }

            online = Gray_IsOnline();
            g_arc_mode = online;                    // lm 字段表示本段是否选择灰度循迹

            if (!online)
            {
                /* 灰度链路离线时，本段改为陀螺仪航向保持。 */
                uint8_t line_lost;
                g_last_line_err = line_error(in->gray_bits, &line_lost);
                straight_steer(in, out, base);
                break;
            }

            err = line_error(in->gray_bits, &lost);  // 灰度在线时，用位图计算循迹误差和可用性
            line_steer(in, out, base, g_seg_yaw_start, 0.0f, 0.0f,
                       err, g_last_line_err, lost, 1);
            g_last_line_err = err;
            break;
        }

        case SEG_ARC:
            arc_step(in, out, seg, base, 0);        // 里程排程弧线供 Q3、Q4 使用
            break;

        case SEG_ARC_LINE:
            /* 灰度掉线后用里程排程弧线作为回退，主要按编码器和 IMU 给出目标。
             * 当前实现仍可把保留的灰度位图用于微调。
             */
            arc_step(in, out, seg, base, Gray_IsOnline());
            break;

        case SEG_TURN:
        {
            float err = g_turn_target_yaw - in->yaw_deg;  // 使用段起点定义的目标角，计算剩余转角
            float v;

            g_turn_ms = (uint16_t)(g_turn_ms + (uint16_t)(in->dt * 1000.0f));

            /* 允许转角进入容差后切段，也允许转向超时后退出本段。 */
            if ((fabsf(err) < TURN_TOL_DEG) || (g_turn_ms > TURN_TIMEOUT_MS))
            {
                if (fabsf(err) < TURN_TOL_DEG)
                {
                    Imu_SetYaw(g_turn_target_yaw);  // 只有实际达到角度容差才校正软件航向
                }
                advance_segment(in, out);
                return;
            }

            v = TURN_KP * fabsf(err);
            v = clampf(v, TURN_MIN_MMPS, TURN_MAX_MMPS);  // 外轮转向速度保持在最小和最大值之间

            if (err > 0.0f)
            {
                /* 左转时右轮为外轮，左轮按 TURN_INNER_RATIO 配速。 */
                out->left_mmps = v * TURN_INNER_RATIO;
                out->right_mmps = v;
            }
            else
            {
                out->left_mmps = v;
                out->right_mmps = v * TURN_INNER_RATIO;
            }
            break;
        }

        case SEG_STOP:
        default:
            g_state = PATH_DONE;
            out->brake = 1;
            out->left_mmps = 0.0f;
            out->right_mmps = 0.0f;
            out->event = PATH_EVENT_STOP;
            return;
    }

    /* 只允许前向目标，双轮目标分别限幅到最高轮速。 */
    if (out->left_mmps < 0.0f)  { out->left_mmps = 0.0f; }
    if (out->right_mmps < 0.0f) { out->right_mmps = 0.0f; }
    if (out->left_mmps > MAX_SPEED_MMPS)  { out->left_mmps = MAX_SPEED_MMPS; }
    if (out->right_mmps > MAX_SPEED_MMPS) { out->right_mmps = MAX_SPEED_MMPS; }
}

/* 路径层的诊断读取接口，供主循环和仿真使用。 */
/**
  * 函    数：获取最近一次计算的灰度线位置误差
  * 参    数：无
  * 返 回 值：归一化误差，范围 -1～+1，正值表示黑线靠 OUT8
  */
float Path_GetLineError(void)
{
    return g_last_line_err;
}

/**
  * 函    数：获取最近计算的循迹或弧段参考航向
  * 参    数：无
  * 返 回 值：航向参考，单位 °
  */
float Path_GetYawRef(void)
{
    return g_yaw_ref_dbg;
}

/**
  * 函    数：获取当前弧线或循迹采用的灰度航向修正
  * 参    数：无
  * 返 回 值：修正角，单位 °
  */
float Path_GetGrayCorr(void)
{
    return g_gray_corr;
}

/**
  * 函    数：获取通过编码器里程与扫过角度估算的弧径
  * 参    数：无
  * 返 回 值：弧线半径估计值，单位 mm，当前段尚无估计时为 0
  */
float Path_GetArcRadius(void)
{
    return g_arc_radius;
}

/**
  * 函    数：获取灰度外环积累的航向偏置
  * 参    数：无
  * 返 回 值：航向偏置，单位 °
  */
float Path_GetLineTheta(void)
{
    return g_line_theta;
}

/**
  * 函    数：获取最近计算的灰度角速度修正
  * 参    数：无
  * 返 回 值：角速度，单位 °/s
  */
float Path_GetLineRate(void)
{
    return g_line_rate_dbg;
}

/**
  * 函    数：获取当前是否选择灰度循迹模式
  * 参    数：无
  * 返 回 值：1 表示选择灰度循迹，0 表示其他模式
  */
uint8_t Path_GetArcMode(void)
{
    return g_arc_mode;
}

/**
  * 函    数：获取弧段结束原因的当前记录
  * 参    数：无
  * 返 回 值：0 为未记录，1 转角保护，2 临近终点无方向信息，3 里程到限，4 经补角流程结束
  */
uint8_t Path_GetArcWhy(void)
{
    return g_arc_why;
}

/**
  * 函    数：读取并消费一份待取弧线统计
  * 参    数：out：用于接收统计的结构体指针
  * 返 回 值：1 表示已复制并清除待取标志，0 表示没有新报告
  */
uint8_t Path_TakeArcReport(ArcReport *out)
{
    if (!g_arc_rep_pending)
    {
        return 0;
    }
    *out = g_arc_rep;                               // 复制本段统计供前台格式化
    g_arc_rep_pending = 0;                          // 报告读取后标记为已消费
    return 1;
}
