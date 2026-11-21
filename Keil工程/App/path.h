#ifndef __PATH_H
#define __PATH_H

#include "stm32f10x.h"

/* 路径状态机的运行状态。 */
typedef enum
{
    PATH_IDLE = 0,                                  // 待机
    PATH_RUNNING,                                   // 正在执行路径
    PATH_DONE                                       // 已完成或因段超时退出
} PathState;

typedef enum
{
    SEG_STOP = 0,
    SEG_STRAIGHT,                                   // MPU 航向保持直线
    SEG_ARC,                                        // 里程推进参考航向，灰度微调
    SEG_LINE,                                       // Q2 的灰度循迹直线
    SEG_ARC_LINE,                                   // Q2 的灰度循迹弧线
    SEG_TURN                                        // 指定转角的转向段
} SegType;

/* 路径层通知声光提示的事件。 */
typedef enum
{
    PATH_EVENT_NONE = 0,
    PATH_EVENT_START,
    PATH_EVENT_POINT,
    PATH_EVENT_STOP
} PathEvent;

/* 路线表中一个路径段的定义。 */
typedef struct
{
    SegType type;
    float   length;                                 // 直线和弧线用里程 mm，转向用角度 °
    float   speed_ratio;                            // 乘到 UI 基础速度上的无量纲倍率
    int8_t  arc_dir;                                // 弧向，+1 左弧，-1 右弧
    uint8_t announce;                               // 本段完成时是否提示到点
} Segment;

/* 每个控制周期传入路径层的传感器快照与设定。 */
typedef struct
{
    float   distance_mm;                            // 双轮平均累计里程，单位 mm
    float   yaw_deg;                                // 当前相对航向，单位 °
    float   gyro_z_dps;                             // 滤波后的陀螺仪 Z 角速度，单位 °/s
    uint8_t gray_bits;                              // 八路黑白位图，1 表示黑
    float   dt;                                     // 本周期的时间长度，单位 s
    float   base_speed_mmps;                        // UI 设定的基础速度，单位 mm/s
} PathInput;

/* 最近一个结束弧段的统计，供前台输出一行串口诊断。 */
typedef struct
{
    int16_t  dist_mm;                               // 弧段相对里程，单位 mm
    int16_t  yaw_deg;                               // 弧段结束时的累计航向，单位 °
    int16_t  radius_mm;                             // 从行驶距离和扫过角度估算的半径，单位 mm
    int16_t  e_min;                                 // 最小归一化灰度误差 × 100
    int16_t  e_max;                                 // 最大归一化灰度误差 × 100
    int8_t   gc_min;                                // 最小灰度航向修正，单位 °
    int8_t   gc_max;                                // 最大灰度航向修正，单位 °
    uint8_t  g_min;
    uint8_t  g_max;
    uint16_t lost;                                  // 全白或全黑的控制周期累计数
    uint8_t  mode;                                  // 1 为灰度循迹，0 为里程排程
    uint8_t  why;                                   // 1 转角保护，2 无方向信息，3 里程到限，4 经补角流程结束
} ArcReport;
/* 路径层本周期输出，交给底层轮速环和提示模块。 */
typedef struct
{
    float     left_mmps;                            // 左轮目标速度，单位 mm/s
    float     right_mmps;                           // 右轮目标速度，单位 mm/s
    uint8_t   brake;                                // 1 请求电机短路制动
    PathEvent event;                                // 本周期路径事件
} PathOutput;

void Path_Init(void);
void Path_Start(uint8_t mode);                      // 1～4 对应 Q1～Q4
void Path_Stop(void);
uint8_t Path_IsRunning(void);
uint8_t Path_GetMode(void);
uint8_t Path_GetSegment(void);
void Path_Update(const PathInput *in, PathOutput *out);

/* 串口诊断和调参读取接口 */
float Path_GetLineError(void);                      // 归一化灰度误差，-1～+1
float Path_GetYawRef(void);                         // 最近一次参考航向，单位 °
float Path_GetGrayCorr(void);                       // 灰度航向修正，单位 °
float Path_GetArcRadius(void);                      // 弧线半径估计值，单位 mm
float Path_GetLineTheta(void);                      // 灰度循迹的航向偏置，单位 °
float Path_GetLineRate(void);                       // 灰度角速度指令，单位 °/s
uint8_t Path_GetArcMode(void);                      // 1 表示采用灰度循迹
uint8_t Path_GetArcWhy(void);                       // 弧段结束原因，见 ArcReport.why
uint8_t Path_TakeArcReport(ArcReport *out);         // 有待取报告时返回 1，并清除待取标志

#endif
