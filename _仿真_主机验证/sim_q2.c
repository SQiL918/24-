/* Q2 循迹主机仿真，直接编译固件的 path.c 和 app_config.h。
 * 差速模型在由两个 1000 mm 直线和两个右转半圆组成的赛道上行驶，名义半径 400 mm。
 * 八路传感器建模在轮轴上，各通道间距 7 mm，OUT1 在左。距离黑线中心小于 10 mm 时判为黑。
 * 传感器每 10 ms 更新，运动控制每 5 ms 执行一次。
 * 轮速命令直接送入简化车体模型，实际 PWM、速度 PI、串口帧和 IMU 读写均由模型接口替代。
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32f10x.h"
#include "app_config.h"
#include "path.h"

#define DT              (0.005f)                    // 仿真步长为 5 ms
#define SENSOR_SPACING  7.0f                        // 相邻灰度探头的距离，单位 mm
#define LINE_HALF_W     10.0f                       // 黑线半宽，单位 mm
#define MOTOR_TAU       0.08f                       // 轮速一阶响应时间常数，单位 s

/* 小车的仿真状态 */
static float p_x, p_y, p_th, p_vl, p_vr, p_dist, p_yaw, p_gyro;

/* 替代固件传感器模块的主机接口 */
static int     s_imu_ready = 1;
static int     s_gray_online = 1;

/**
  * 函    数：从模型返回软件积分航向
  * 参    数：无
  * 返 回 值：仿真航向，单位 °
  */
float   Imu_GetYaw(void)          { return p_yaw; }
/**
  * 函    数：获取差速模型计算的 Z 轴角速度
  * 参    数：无
  * 返 回 值：角速度，单位 °/s
  */
float   Imu_GetGyroZ(void)        { return p_gyro; }
/**
  * 函    数：修改软件航向模型中的积分值
  * 参    数：y：目标软件航向，单位 °
  * 返 回 值：无，物理朝向 p_th 保持原值
  */
void    Imu_SetYaw(float y)       { p_yaw = y; }
/**
  * 函    数：清零模型的航向和陀螺仪读数
  * 参    数：无
  * 返 回 值：无
  */
void    Imu_ResetYaw(void)        { p_yaw = 0.0f; p_gyro = 0.0f; }
/**
  * 函    数：查询场景设置的 IMU 就绪标志
  * 参    数：无
  * 返 回 值：1 为就绪，0 为未就绪
  */
uint8_t Imu_IsReady(void)         { return (uint8_t)s_imu_ready; }
/**
  * 函    数：查询场景设置的灰度在线标志
  * 参    数：无
  * 返 回 值：1 为在线，0 为离线
  */
uint8_t Gray_IsOnline(void)       { return (uint8_t)s_gray_online; }

/* Q2 赛道几何与距离计算 */
/**
  * 函    数：计算点到有限长直线段的最近距离
  * 参    数：px：待测点 X 坐标
  *           py：待测点 Y 坐标
  *           ax：线段起点 X 坐标
  *           ay：线段起点 Y 坐标
  *           bx：线段终点 X 坐标
  *           by：线段终点 Y 坐标
  * 返 回 值：欧氏距离，单位 mm
  */
static float seg_dist(float px, float py, float ax, float ay, float bx, float by)
{
    float vx = bx - ax, vy = by - ay;
    float wx = px - ax, wy = py - ay;
    float t = (vx * wx + vy * wy) / (vx * vx + vy * vy);  // 把待测点投影到承载线段的直线上
    float dx, dy;

    /* 将投影比例限制在 0～1，超出线段时改用最近端点。 */
    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }
    dx = px - (ax + t * vx);
    dy = py - (ay + t * vy);
    return sqrtf(dx * dx + dy * dy);
}

/* right 仅表示半圆所在的 X 方向。1 为圆心右侧的第一段弧，0 为左侧的第二段弧。 */
/**
  * 函    数：在点位于指定半圆的角域时计算径向距离
  * 参    数：px：待测点 X 坐标
  *           py：待测点 Y 坐标
  *           cx：圆心 X 坐标
  *           cy：圆心 Y 坐标
  *           r：半圆半径，单位 mm
  *           right：1 选择 X 正侧半圆，0 选择 X 负侧半圆
  * 返 回 值：径向偏差，单位 mm，超出半圆角域时返回很大的哨兵值
  */
static float arc_dist(float px, float py, float cx, float cy, float r, int right)
{
    float dx = px - cx, dy = py - cy;
    float ang = atan2f(dy, dx);
    int in_span;

    in_span = right ? (fabsf(ang) <= 1.5707963f) : (fabsf(ang) >= 1.5707963f);  // 判断点是否位于选定的半圆角域
    if (in_span)
    {
        return fabsf(sqrtf(dx * dx + dy * dy) - r);  // 点到圆心的距离减半径，再取绝对值
    }
    return 1.0e9f;                                  // 投影不在半圆角域，由其他赛道段提供距离
}

/* 场景可改动赛道真实半径，用来比较固件名义半径为 400 mm 时的误差。 */
static float g_R = 400.0f;

/**
  * 函    数：计算到两段直线和两段半圆的最小距离
  * 参    数：px：查询点 X 坐标
  *           py：查询点 Y 坐标
  * 返 回 值：到赛道中心线的最近距离，单位 mm
  */
static float track_dist(float px, float py)
{
    float d = seg_dist(px, py, 0.0f, 0.0f, 1000.0f, 0.0f);  // 从 A→B 第一段直线的距离开始比较
    float t;

    t = arc_dist(px, py, 1000.0f, -g_R, g_R, 1);    // 比较右侧半圆 B→C
    if (t < d) { d = t; }
    t = seg_dist(px, py, 1000.0f, -2.0f * g_R, 0.0f, -2.0f * g_R);  // 比较第二段直线 C→D
    if (t < d) { d = t; }
    t = arc_dist(px, py, 0.0f, -g_R, g_R, 0);       // 比较左侧半圆 D→A
    if (t < d) { d = t; }
    return d;
}

/* 在车轴横向摆放八个采样点，从左至右生成黑白位图。 */
/**
  * 函    数：根据车身位置和方向生成八路灰度传感器位图
  * 参    数：x：车轴中心 X 坐标，单位 mm
  *           y：车轴中心 Y 坐标，单位 mm
  *           th：车身方向相对 X 轴的角度，单位 rad
  * 返 回 值：八路黑白位图，位值 1 表示压线
  */
static uint8_t sensor_bits(float x, float y, float th)
{
    uint8_t bits = 0;
    int i;

    for (i = 0; i < 8; i++)
    {
        float s = -24.5f + SENSOR_SPACING * (float)i;  // 第 i 路相对阵列中心的横向距离，正值为车体右侧
        float rx = sinf(th);
        float ry = -cosf(th);
        float px = x + s * rx;                      // 将横向偏移投影到地面 X 坐标
        float py = y + s * ry;                      // 将横向偏移投影到地面 Y 坐标

        if (track_dist(px, py) < LINE_HALF_W)       // 最近赛道中心线距离小于线宽一半即判黑
        {
            bits |= (uint8_t)(1u << i);
        }
    }
    return bits;
}

/* 双轮差速车体模型 */
/**
  * 函    数：设置仿真车辆的初始位姿、里程和反馈状态
  * 参    数：x0：初始 X 坐标，单位 mm
  *           y0：初始 Y 坐标，单位 mm
  *           th0_deg：物理起跑方向，单位 °
  *           dist0：初始累计里程，单位 mm
  * 返 回 值：无
  */
static void plant_reset(float x0, float y0, float th0_deg, float dist0)
{
    p_x = x0; p_y = y0; p_th = th0_deg * 0.017453293f;  // 物理角度从度换为弧度
    p_vl = 0.0f; p_vr = 0.0f;
    p_dist = dist0; p_yaw = 0.0f; p_gyro = 0.0f;    // 仿真软件航向按起跑方向重新归零
}

/**
  * 函    数：用一阶轮速响应和差速运动学推进一帧
  * 参    数：l：左轮目标速度，单位 mm/s
  *           r：右轮目标速度，单位 mm/s
  * 返 回 值：无
  */
static void plant_step(float l, float r)
{
    float k = DT / (MOTOR_TAU + DT);                // 将连续一阶惯性离散到 5 ms 步进
    float v;

    /* 左、右轮分别用同一时间常数向目标速度靠近。 */
    p_vl += (l - p_vl) * k;
    p_vr += (r - p_vr) * k;

    p_gyro = (p_vr - p_vl) / CAR_TRACK_WIDTH_MM * 57.29578f;  // 轮速差除以轮距得到角速度，再换成 °/s
    p_yaw += p_gyro * DT;                           // 更新软件航向，它可被 Path 层校正
    p_th += p_gyro * 0.017453293f * DT;             // 独立积分物理朝向，单位 rad

    v = 0.5f * (p_vl + p_vr);                       // 左右轮速平均值作为车身前进速度
    p_x += v * cosf(p_th) * DT;
    p_y += v * sinf(p_th) * DT;
    p_dist += v * DT;                               // 按当前前进速度累积直线等效里程
}

/* 单个仿真场景的运行和统计 */
/**
  * 函    数：用真实路径状态机驱动车体模型并记录结果
  * 参    数：mode：路径模式编号
  *           base：基础速度，单位 mm/s
  *           y0：起点横向偏差，单位 mm
  *           th0：物理起跑方向，单位 °
  *           csv：轨迹输出文件名，空指针表示不输出
  *           out_maxdev：写入最大轨迹偏差的指针，单位 mm
  *           out_time：写入仿真用时的指针，单位 s
  *           out_yaw：写入结束软件航向的指针，单位 °
  *           drop_from：模拟灰度掉线起始节拍
  *           drop_to：掉线结束节拍，0 表示不模拟掉线
  * 返 回 值：1 表示在仿真总时限前退出路线运行状态，0 表示达到总时限
  */
static int run(int mode, float base, float y0, float th0, const char *csv,
               float *out_maxdev, float *out_time, float *out_yaw,
               int drop_from, int drop_to)
{
    PathInput in;
    PathOutput out;
    uint8_t bits = 0;
    int tick;
    int tick_max = 12000;                           // 12000 个控制周期等于 60 s
    int done_ticks = 0;
    float maxdev = 0.0f;
    FILE *f = 0;
    int arc = 0;
    int gray_ok = s_gray_online;

    plant_reset(0.0f, y0, th0, 0.0f);               // 从场景指定的位姿起跑，里程归零
    Path_Init();
    Path_Start((uint8_t)mode);                      // 启动指定任务模式

    bits = sensor_bits(p_x, p_y, p_th);

    if (csv)
    {
        f = fopen(csv, "w");
        if (f) { fprintf(f, "t,x,y,th,yaw,e,d,bits,lt,l,r\n"); }
    }

    for (tick = 0; tick < tick_max; tick++)
    {
        float dev;
        float l, r;

        if (drop_to > 0 && tick >= drop_from && tick < drop_to)
        {
            s_gray_online = 0;                      // 模拟链路离线，仅修改在线标志
        }
        else
        {
            s_gray_online = gray_ok;
        }

        if ((tick & 1) == 0)
        {
            bits = sensor_bits(p_x, p_y, p_th);     // 每两个控制步重新建模一次八路传感器，掉线场景也会刷新位图
        }

        in.distance_mm = p_dist;
        in.yaw_deg = p_yaw;
        in.gyro_z_dps = p_gyro;
        in.gray_bits = bits;
        in.dt = DT;
        in.base_speed_mmps = base;

        Path_Update(&in, &out);                     // 执行固件的实际路径状态机

        {
            ArcReport rep;
            if (Path_TakeArcReport(&rep))           // 取出刚结束的一段弧线汇总
            {
                printf("  t=%.2f ARC %d: d=%d yaw=%d Rr=%d lm=%u e=%d..%d gc=%d..%d "
                       "g=%u..%u lost=%u why=%u\n", tick*DT, arc++, (int)rep.dist_mm,
                       (int)rep.yaw_deg, (int)rep.radius_mm, rep.mode,
                       rep.e_min, rep.e_max, rep.gc_min, rep.gc_max,
                       rep.g_min, rep.g_max, rep.lost, rep.why);
            }
        }

        if (out.brake)                              // 制动命令在模型中等效为两轮目标速度均置零
        {
            l = 0.0f; r = 0.0f;
        }
        else
        {
            l = out.left_mmps;
            r = out.right_mmps;
        }

        plant_step(l, r);                           // 用本周期的目标轮速推进车体和里程

        dev = track_dist(p_x, p_y);
        if (tick > 100 && dev > maxdev) { maxdev = dev; }  // 忽略头 101 个采样点，仅统计约 0.5 s 后的偏差

        if (f)
        {
            fprintf(f, "%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.1f,%u,%.1f,%.1f,%.1f\n",
                    tick * DT, p_x, p_y, p_th * 57.29578f, p_yaw,
                    (double)Path_GetLineError(), p_dist, bits,
                    (double)Path_GetLineTheta(), l, r);
        }

        if (!Path_IsRunning())
        {
            done_ticks++;
            if (done_ticks > 40) { break; }         // 任务退出运行状态后再模拟约 0.2 s，观察惯性
        }
    }

    if (f) { fclose(f); }

    *out_maxdev = maxdev;
    *out_time = tick * DT;
    *out_yaw = p_yaw;

    return (tick < tick_max) ? 1 : 0;               // 只判断是否在总时限前停止运行，段超时也可能被标成 DONE
}

/**
  * 函    数：依次运行 Q2 场景并打印一行结果汇总
  * 参    数：无
  * 返 回 值：进程正常执行完场景列表后返回 0
  */
int main(void)
{
    struct
    {
        const char *name;
        int mode;
        float base;
        float y0;
        float th0;
        int gray;
        const char *csv;
        int drop_from;
        int drop_to;
        float r;
    } cases[] =
    {
        /* 场景覆盖不同弧径、横向或角度偏置、灰度掉线以及不同基础速度。 */
        { "Q2 600 centred  R=400",        2, 600.0f,   0.0f, 0.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 wider arc R=430",       2, 600.0f,   0.0f, 0.0f, 1, 0, 0, 0, 430.0f },
        { "Q2 600 tight arc R=370",       2, 600.0f,   0.0f, 0.0f, 1, 0, 0, 0, 370.0f },
        { "Q2 450 wider arc R=450",       2, 450.0f,   0.0f, 0.0f, 1, 0, 0, 0, 450.0f },
        { "Q2 600 10 mm",                 2, 600.0f,  10.0f, 0.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 20 mm",                 2, 600.0f,  20.0f, 0.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 30 mm (lost at start)", 2, 600.0f,  30.0f, 0.0f, 1, "traj_q2_30mm.csv", 0, 0, 400.0f },
        { "Q2 600 3 deg",                 2, 600.0f,   0.0f, 3.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 10mm/3deg",             2, 600.0f,  10.0f, 3.0f, 1, 0, 0, 0, 400.0f },
        { "Q2 600 link drop 2..4s",       2, 600.0f,   0.0f, 0.0f, 1, 0, 400, 800, 400.0f },
        { "Q2 600 NO gray (fallback)",    2, 600.0f,   0.0f, 0.0f, 0, 0, 0, 0, 400.0f },
        { "Q2 450 centred  R=400",        2, 450.0f,   0.0f, 0.0f, 1, "traj_q2_450.csv", 0, 0, 400.0f },
        { "Q2 900 centred  R=400",        2, 900.0f,   0.0f, 0.0f, 1, 0, 0, 0, 400.0f }
    };
    unsigned i;

    printf("path.c host simulation\n");
    printf("%-32s %-8s %6s %8s %8s\n", "case", "result", "time", "maxdev", "yaw");

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        float dev, tm, yaw;
        int ok;

        s_gray_online = cases[i].gray;
        s_imu_ready = 1;
        g_R = cases[i].r;                           // 选用该场景的仿真赛道半径

        ok = run(cases[i].mode, cases[i].base, cases[i].y0, cases[i].th0,
                 cases[i].csv, &dev, &tm, &yaw,
                 cases[i].drop_from, cases[i].drop_to);

        printf("%-32s %-8s %5.2fs %7.1fmm %8.1f  end=(%.0f,%.0f)\n",
               cases[i].name, ok ? "DONE" : "TIMEOUT", tm, dev, yaw, p_x, p_y);
    }
    return 0;
}


