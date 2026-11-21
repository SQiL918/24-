#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "stm32f10x.h"

/* 第十六队 STM32F103C8T6 循迹小车参数。
 * 硬件为 STM32 Blue Pill、TB6612FNG、两只 MG513 编码器电机、八路 CH32V006 灰度模块、MPU6050 和 SSD1306 OLED。
 *
 * 引脚分配。改动时同步核对接线表「接线图/接线总表.md」。
 * PA0  KEY_START       上拉输入，低电平为按下
 * PA1  KEY_SPEED       上拉输入，低电平为按下
 * PA2  TIM2_CH3/PWMA   左轮电机 PWM
 * PA3  TIM2_CH4/PWMB   右轮电机 PWM
 * PA4/PA5  AIN1/AIN2   左轮方向控制
 * PA6/PA7  TIM3_CH1/2   左轮编码器 A/B
 * PA8/PA11/PA12/PA15   原灰度 OUT4～OUT7，当前下拉闲置
 * PA9  USART1_TX       同时连接调试 CH340 RX 与灰度模块 RX
 * PA10 USART1_RX       只连接灰度模块 TX
 * PA13/PA14            SWDIO/SWCLK，留给 ST-Link
 * PB0  BUZZER          经 S8050 驱动有源蜂鸣器
 * PB1  ADC_IN9         电池电压分压输入
 * PB2  LED             外置状态灯
 * PB3/PB4/PB5/PB14     原灰度 OUT1、OUT2、OUT3、OUT8，当前下拉闲置
 * PB6/PB7  TIM4_CH1/2   右轮编码器 A/B
 * PB8/PB9              OLED 软件 I²C 的 SCL/SDA
 * PB10/PB11            MPU6050 软件 I²C 的 SCL/SDA
 * PB12/PB13 BIN1/BIN2  右轮方向控制
 * PB15 KEY_MODE        上拉输入，低电平为按下
 * PC13 LED_ONBOARD     板载指示灯，低电平点亮
 *
 * Q2-GRAY-UART 版本使用 115200 8N1 读取灰度串口。
 * 不应把 CH340 TX 也接到 PA10。灰度模块原来的八路并行 OUT 排线需断开。
 */

/* 板级引脚定义 */
#define KEY_START_PORT      GPIOA
#define KEY_START_PIN       GPIO_Pin_0
#define KEY_SPEED_PORT      GPIOA
#define KEY_SPEED_PIN       GPIO_Pin_1
#define KEY_MODE_PORT       GPIOB
#define KEY_MODE_PIN        GPIO_Pin_15

#define LED_PORT            GPIOB
#define LED_PIN             GPIO_Pin_2
#define LED_ONBOARD_PORT    GPIOC
#define LED_ONBOARD_PIN     GPIO_Pin_13

#define BUZZER_PORT         GPIOB
#define BUZZER_PIN          GPIO_Pin_0

#define MOTOR_LEFT_AIN1_PORT GPIOA
#define MOTOR_LEFT_AIN1_PIN  GPIO_Pin_4
#define MOTOR_LEFT_AIN2_PORT GPIOA
#define MOTOR_LEFT_AIN2_PIN  GPIO_Pin_5
#define MOTOR_RIGHT_BIN1_PORT GPIOB
#define MOTOR_RIGHT_BIN1_PIN  GPIO_Pin_12
#define MOTOR_RIGHT_BIN2_PORT GPIOB
#define MOTOR_RIGHT_BIN2_PIN  GPIO_Pin_13

/* 赛道与车辆几何，单位为 mm 或 ° */
#define TRACK_AB_MM         1000.0f                 // A→B 直线长度，单位 mm
#define TRACK_CD_MM         1000.0f                 // C→D 直线长度，单位 mm
#define TRACK_ARC_MM        1256.637f               // 半圆长度，π × 400 mm
#define TRACK_DIAG_MM       1280.625f               // 对角线长度，√(1000² + 800²) mm
#define TRACK_ANGLE_DEG     38.6598f                // 对角线与长边夹角，atan2(800, 1000)，单位 °
#define TRACK_RADIUS_MM     400.0f                  // 赛道名义半圆半径，单位 mm
#define CAR_TRACK_WIDTH_MM  150.0f                  // 左右轮中心之间的距离，单位 mm
#define TURN_FORWARD_MM     47.0f                   // 38.66° 单轮转向的估算前移量，单位 mm
#define TURN_INNER_RATIO    0.0f                    // 内轮目标速度与外轮之比，0 表示内轮制动

/* 编码器与车轮标定 */
/* MG513 按 11 个脉冲 × 四倍频 × 30 减速比，计算为每轮转 1320 计数。
 * 65 mm 轮径对应约 204.2 mm 周长、6.464 计数/mm。可手推 1000 mm 复测并校准。
 */
#define ENCODER_COUNTS_PER_REV  1320.0f             // 减速器输出轴每转的编码器总计数
#define WHEEL_DIAMETER_MM       65.0f               // 车轮名义直径，单位 mm
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265f * WHEEL_DIAMETER_MM)  // 按轮径计算每转行驶距离
#define COUNTS_PER_MM           (ENCODER_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_MM)  // 编码器计数换算为里程的比例
/* 2026-11-07 台架记录，D153B 左轮接电机 A，右轮接电机 B。
 * 电机方向保持默认时车向前运行。左编码器取反时前进曾读到 L=-1421，当前按两轮前进计数均为正配置。
 */
#define ENCODER_LEFT_SIGN       (+1)                // 左编码器方向因子
#define ENCODER_RIGHT_SIGN      (+1)                // 右编码器方向因子
#define MOTOR_LEFT_INVERT       0                   // 左电机驱动方向，1 表示反相
#define MOTOR_RIGHT_INVERT      0                   // 右电机驱动方向，1 表示反相
/* 期望逆时针左转时航向增大。若实测减小，可将 GYRO_Z_SIGN 改为 -1。 */
#define GYRO_Z_SIGN             (+1.0f)

/* 控制周期与电机输出上限 */
#define CONTROL_HZ              200.0f              // 控制频率，单位 Hz，须与 TIM1 定时配置一致
#define CONTROL_DT              (1.0f / CONTROL_HZ)  // 控制步长，单位 s，当前为 0.005 s
#define PWM_MAX                 3599                // TIM2 自动重装值，72 MHz 时约为 20 kHz PWM
#define PWM_FREQ_HZ             20000               // 名义 PWM 频率，当前 TIM2 由 PWM_MAX 决定
#define MAX_SPEED_MMPS          1100.0f             // 左右轮目标的最高速度，单位 mm/s
#define SPEED_MIN_MMPS          0.0f                // 名义最低轮速，单位 mm/s
#define SPEED_RAMP_MMPS2        1800.0f             // 预留加速度斜坡参数，现有控制未引用，单位 mm/s²

/* 单轮速度 PI 与前馈 */
#define SPEED_KP                1.10f               // 轮速比例增益
#define SPEED_KI                18.0f               // 轮速积分增益，积分时再乘控制周期
#define SPEED_KFF               (3599.0f / 1100.0f)  // 将目标轮速映射到初始 PWM 比较值
#define SPEED_I_LIMIT           450.0f              // 轮速积分贡献的正、负上限
#define SPEED_DEADBAND          90.0f               // 正向 PWM 输出的死区补偿量

/* MPU 直线航向保持 */
#define YAW_KP                  8.0f                // 航向误差每偏 1° 对应的差速补偿，单位 (mm/s)/°
#define YAW_KD                  1.5f                // 角速度阻尼增益，单位 (mm/s)/(°/s)
#define YAW_TURN_LIMIT          220.0f              // 直线航向修正的差速上限，单位 mm/s

/* 里程排程弧线，供 Q3、Q4 和 Q2 灰度离线时使用。
 * turn_ff = arc_dir × base × CAR_TRACK_WIDTH_MM / (2 × TRACK_RADIUS_MM) × ARC_FF_TRIM。
 * gray_corr = -(ARC_GRAY_TRIM_KP × e + ARC_GRAY_TRIM_KD × 本周期误差差值)。
 * yaw_ref = 弧线起点角 + arc_dir × 180 × 里程进度 + gray_corr。
 * rate_ff = arc_dir × 180 × base / 弧长。航向内环使用实际角速度与 rate_ff 的差值，避免稳定转弯时产生偏置。
 * 最终差速为曲率前馈加闭环修正，左轮用基础速度减差速，右轮加差速。
 */

/* 曲率前馈标定。可用行驶弧长除以扫过角度（弧度）估算实际半径 R_act。
 * 如果实际半径偏大，增加 ARC_FF_TRIM。若偏小，减小该倍率。可用 R_act / TRACK_RADIUS_MM 作参考。
 */
#define ARC_FF_TRIM             1.00f               // 固定曲率前馈的标定倍率

/* 里程排程弧使用 -1～+1 的归一化灰度误差调整参考航向。
 * 增益输出是角度，八路二值位图导致灰度误差呈离散变化。
 */
#define ARC_GRAY_TRIM_KP        20.0f               // 里程排程弧的灰度比例增益，单位 °/归一化误差
#define ARC_GRAY_TRIM_KD        4.0f                // 里程排程弧的误差差值增益，按相邻 5 ms 样本定义
#define ARC_GRAY_CORR_MAX       40.0f               // 灰度对参考航向的最大修正，单位 °
#define ARC_REACQ_DEG           25.0f               // 里程排程弧丢线时朝历史方向回找的偏置，单位 °
#define ARC_GRAY_LOST_DECAY     0.97f               // 每 5 ms 的丢线偏置衰减系数

/* Q2 灰度循迹控制，用于 SEG_LINE 和 SEG_ARC_LINE。C→D 仍是 MPU 直线。
 * 直线的曲率前馈为零，弧线使用 400 mm 名义半径。误差 e 为正表示黑线靠 OUT8。
 * w_line = -(KP × e + KD × 低通后的误差变化率)，输出单位 °/s，按 W_MAX 限幅。
 * theta_line 按 w_line × dt 累积，单位 °，按 THETA_MAX 限幅。
 * yaw_ref = 路线切线航向 + theta_line，rate_ref = 曲率角速度前馈 + w_line。
 * 航向内环用角度和角速度反馈计算差速，叠加曲率前馈后分配给左右轮。
 * 调参时看串口 e 和 lt。长时间切内或跑外可适当增大 KP，摆动时先适当减小 KP 再看 KD。
 * 外环角速度过于激烈可降低 W_MAX，入弯纠偏幅度不足时核查 THETA_MAX。
 */
#define ARC_LINE_KP             60.0f               // 每单位灰度误差对应的目标角速度修正
#define ARC_LINE_KD             20.0f               // 灰度误差变化率的反馈增益
#define ARC_LINE_D_FILT         0.20f               // 微分低通的新样本权重，控制周期 5 ms
#define ARC_LINE_W_MAX          110.0f              // 灰度外环角速度指令的幅值上限，单位 °/s
#define ARC_LINE_THETA_MAX      55.0f               // 灰度外环积分航向偏置的幅值上限，单位 °
#define ARC_LINE_REACQ_DEG      40.0f               // 灰度闭环丢线后尝试回找的航向偏置，单位 °

/* 弧线与灰度循迹使用的 MPU 航向内环 */
#define ARC_YAW_KP              10.0f               // 航向误差比例增益，单位 (mm/s)/°
#define ARC_YAW_KD              0.8f                // 角速度误差反馈增益，单位 (mm/s)/(°/s)
#define ARC_CORR_LIMIT          200.0f              // 闭环差速部分的幅值上限，单位 mm/s

/* IMU 未就绪时使用曲率前馈加灰度直接差速修正。 */
#define ARC_LINE_FALLBACK_KP    120.0f              // 无航向反馈时的灰度差速增益，单位 (mm/s)/归一化误差

/* 单轮转向控制 */
#define TURN_KP                 5.0f                // 根据剩余角度求外轮速度，单位 (mm/s)/°
#define TURN_MIN_MMPS           130.0f              // 外轮目标的最低起转速度，单位 mm/s
#define TURN_MAX_MMPS           520.0f              // 外轮目标的最高转向速度，单位 mm/s
#define TURN_TOL_DEG            1.8f                // 普通转向段到位容差，单位 °
#define TURN_TIMEOUT_MS         2500                // 普通转向或出弯补角的超时，单位 ms

/* 弧线与分段结束判定 */
#define ARC_LINE_LOST_SAMPLES   3                   // 弧末结束判定要求的连续无方向采样数
#define ARC_FINISH_MARGIN_MM    5.0f                // 名义弧长之外允许的收尾里程，单位 mm，旧值为 60 mm
#define ARC_YAW_DONE_DEG        179.0f              // 接近弧末时的过转保护阈值，单位 °
#define ARC_YAW_SNAP_DEG        3.0f                // 允许软件航向吸附到目标的最大残差，单位 °
#define ARC_YAW_FIX_MAX         20.0f               // 排程弧允许将目标交给下一直线的残差上限，单位 °，灰度弧另有 45° 分支
#define SEGMENT_TIMEOUT_MS      15000               // 单个路径段运行的超时时限，单位 ms

/* Q2 灰度弧结束并且下一段为直线时，可先停车再绕内轮补足航向差。
 * 差值不超过触发阈值时直接收尾。达到到位阈值时修正软件航向。
 * 补角使用 TURN_KP、TURN_MIN_MMPS、TURN_MAX_MMPS 与 TURN_TIMEOUT_MS。
 */
#define ARC_SETTLE_TOL          1.0f                // 触发出弯补角的航向残差阈值，单位 °
#define ARC_SETTLE_DONE_TOL     1.0f                // 判定补角到位的残差容差，单位 °
#define ARC_SETTLE_BRAKE_MS     150                 // 转向前先制动等待的时长，单位 ms

/* 交接目标航向的参考选项。
 * 1 表示按起跑方向逐弧累加 ±180°，起跑摆放角误差仍可能传给下一直线。
 * 0 表示按本弧段的入口航向转 ±180°。
 */
#define ARC_SETTLE_ABS_YAW      1

/* CH32V006 八路灰度模块串口简化协议，115200 8N1。
 * 请求 A5 CMD ARGS，响应 5A CMD STATUS PAYLOAD。
 * 0x02 读寄存器，0x03 写寄存器，0x05 读八路灰度值（计数加 8 个小端 16 位值）。
 * 寄存器 0x04 是黑白比较位图，bit0 对应通道 1。0x13/0x14 为比较阈值，默认 2000。
 * 0x15 控制推送模式，0 关闭、1 推位图、2 推八路原始 ADC。
 * 启动后发送 A5 03 15 02，每约 10 ms 接收一帧。
 * 帧格式为 5B + g1L g1H … g8L g8H，共 17 字节，各值为 12 位。
 * 接收器寻找 0x5B 帧头，校验高字节并在错帧后重新同步。应用按 GRAY_BLACK_ADC 生成黑白位图。
 */
#define GRAY_UART_BAUD          115200              // 灰度模块和调试输出共用的串口波特率
#define GRAY_BLACK_ADC          2000                // 原始灰度判黑阈值
#define GRAY_LINK_TIMEOUT_MS    300                 // 无有效帧超过该时长则离线，单位 ms
#define GRAY_LINK_RETRY_MS      200                 // 两次发送推送配置命令的最小间隔，单位 ms

/* 灰度原值是 12 位。若探头左右方向与位图约定相反，将 GRAY_REVERSE 设为 1。 */
#define GRAY_BLACK_IS_HIGH      1                   // 1 表示超过阈值判黑
#define GRAY_REVERSE            0                   // 1 表示将 8 个通道的位序反转

/* 若有探头常黑，可在白底检查串口 a 字段定位。正常白底通常为几百计数。
 * 若某一路接近 4095 且不变化，可用位掩码临时屏蔽，例如第 6 路用 0x20u。
 */
#define GRAY_MASK               0x00u               // 置位即忽略对应通道，bit0 对应当前位图的第 1 路

/* 电池电压采样与标定 */
/* 电池为 WHEELTEC E326S 三节串联，标称使用范围 9.0～12.6 V。
 * D153B 的 Output_IO ADC 脚连接 PB1，板载分压名义上为 1/11。ADC 以 3.3 V 为参考做 12 位采样。
 * 若改用 100 kΩ/22 kΩ 外部分压，倍率约为 5，最终应以万用表实测标定。
 */
/* 2026-11-07 实测电池 12.5 V，倍率 11 时 OLED 显示 13.7 V，当时将倍率修正到约 10。
 * 这一差异可能来自实际分压或模拟电源参考电压。
 */
/* 2026-11-08 电机停止时实测 12.4 V，倍率 10 时显示约 15.55 V。
 * 以 10 × 12.4 / 15.55 估算约 7.97，当前显示和日志采用倍率 8。
 */
#define VBAT_DIVIDER_RATIO      (1.0f / 11.0f)      // 名义分压比，当前显示计算未引用
#define VBAT_ADC_MULT           8UL                 // 电池电压 = ADC 引脚换算电压 × 实测标定倍率
#define VBAT_LOW_MV             10200               // 低电压参考值，单节 3.40 V，目前未用于停车控制
#define VBAT_CRITICAL_MV        9600                // 临界电压参考值，单节 3.20 V，目前未用于停车控制

/* 看门狗与故障诊断 */
/* 主程序在板级初始化后、上电校准前开启独立看门狗。
 * 正常校准与主循环会喂狗。看门狗复位后串口输出 IWDG 复位标记。
 * 超时设为 0 时不启用。
 */
#define IWDG_TIMEOUT_MS         2000                // 独立看门狗名义超时，单位 ms

/* 界面与速度档位 */
/* 固件启动时输出版本字符串，便于从日志确认烧录版本。 */
#define FW_TAG                  "Q2-GRAY-UART r8"
#define SPEED_SET_MIN           300                 // 最低基础速度档位，单位 mm/s
#define SPEED_SET_MAX           900                 // 最高基础速度档位，单位 mm/s
#define SPEED_SET_STEP          150                 // 每次按速度键的步进，单位 mm/s
#define UI_REFRESH_MS           200                 // OLED 刷新间隔，单位 ms

#endif
