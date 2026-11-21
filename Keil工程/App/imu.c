#include "stm32f10x.h"
#include "imu.h"
#include "app_config.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "Delay.h"
#include "bsp.h"

#define MPU6050_ADDR_W      0xD0                    // 7 位器件地址 0x68 对应的写地址字节
#define MPU6050_ADDR_R      0xD1                    // 7 位器件地址 0x68 对应的读地址字节
#define GYRO_LSB_PER_DPS    16.4f                   // 陀螺仪量程 ±2000 °/s 时的标度，每 °/s 为 16.4 计数

static float g_gyro_bias_raw = 0.0f;                // 静止校准得到的 Z 轴零偏，单位为原始计数
static float g_gyro_z_dps = 0.0f;                   // 供航向环使用的滤波角速度，单位 °/s
static float g_yaw_deg = 0.0f;                      // 相对航向角，单位 °
static uint8_t g_imu_id = 0;
static uint8_t g_imu_ok = 0;

/**
  * 函    数：连续读取 MPU6050 陀螺仪 Z 轴高、低字节
  * 参    数：无
  * 返 回 值：有符号 16 位原始值，地址阶段未获应答时返回 0
  */
static int16_t Imu_ReadGyroZRaw(void)
{
    uint8_t h;
    uint8_t l;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR_W);
    if (MyI2C_ReceiveAck() != 0)                    // 地址无应答，本次读数按 0 处理
    {
        MyI2C_Stop();
        return 0;
    }
    MyI2C_SendByte(MPU6050_GYRO_ZOUT_H);            // 选中 Z 轴高字节寄存器，低字节在下一地址
    MyI2C_ReceiveAck();

    MyI2C_Start();                                  // 重复 START，保持寄存器读取事务连续
    MyI2C_SendByte(MPU6050_ADDR_R);
    if (MyI2C_ReceiveAck() != 0)                    // 读地址阶段失败时提前结束
    {
        MyI2C_Stop();
        return 0;
    }
    h = MyI2C_ReceiveByte();                        // 先读高字节
    MyI2C_SendAck(0);                               // 发送 ACK，请从机继续发送低字节
    l = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);                               // 发送 NACK，结束两字节读取
    MyI2C_Stop();

    return (int16_t)(((uint16_t)h << 8) | (uint16_t)l);  // 按寄存器字节顺序合成为有符号 16 位数据
}

/**
  * 函    数：初始化 MPU6050，配置采样并检查唤醒状态
  * 参    数：无
  * 返 回 值：无，就绪状态由 Imu_IsReady 读取
  */
void Imu_Init(void)
{
    uint8_t try;
    uint8_t pwr = 0xFF;

    MyI2C_Init();

    /* 冷上电时 MPU6050 需要稳定时间。过早写寄存器可能失败，导致 WHO_AM_I 正常但芯片仍在休眠。
     * 先等待供电稳定，再写配置、读回休眠位并按需重试。
     */
    Delay_ms(100);

    g_imu_id = 0;
    g_imu_ok = 0;

    for (try = 0; try < 5; try++)
    {
        BSP_WdgFeed();                              // 初始化包括正常延时，重试前喂狗
        g_imu_id = MPU6050_GetID();
        if (g_imu_id == 0x00 || g_imu_id == 0xFF)
        {
            Delay_ms(100);                          // 芯片暂未正确应答，等待后重试
            continue;
        }

        /* 退出休眠，以 X 轴陀螺仪 PLL 作时钟源。 */
        MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
        MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);  // 清除各轴待机位

        /* 将采样率设为 1000 / (1 + 4) = 200 Hz。 */
        MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);
        MPU6050_WriteReg(MPU6050_CONFIG, 0x03);     // 选择数字低通档位 3，陀螺仪带宽 42 Hz

        /* 设置陀螺仪量程 ±2000 °/s，加速度计设为 ±2 g。 */
        MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
        MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);  // 当前航向算法不使用加速度数据

        Delay_ms(20);                               // 等待电源管理配置生效再检查

        pwr = MPU6050_ReadReg(MPU6050_PWR_MGMT_1);
        if ((pwr & 0x40) == 0)                      // 休眠位清零表示已唤醒
        {
            g_imu_ok = 1;
            break;
        }

        Delay_ms(100);                              // 仍处于休眠，延时后重配
    }

    BSP_UartSendString("\r\nMPU6050 WHO_AM_I=");
    BSP_UartSendInt((int32_t)g_imu_id);
    BSP_UartSendString(" PWR_MGMT_1=");
    BSP_UartSendInt((int32_t)pwr);
    BSP_UartSendString(g_imu_ok ? " (ok, expect ID 104)\r\n"
                                : " (FAIL, expect ID 104)\r\n");
}

/**
  * 函    数：车体静止时采集 400 次 Z 轴原始值，估计陀螺仪零偏
  * 参    数：无
  * 返 回 值：无
  */
void Imu_Calibrate(void)
{
    int32_t sum = 0;
    int16_t raw;
    int16_t raw_min = 32767;
    int16_t raw_max = -32768;
    uint16_t i;

    if (!g_imu_ok)                                  // 初始化未确认唤醒时跳过校准
    {
        BSP_UartSendString("IMU not found, skip gyro calibration\r\n");
        return;
    }

    for (i = 0; i < 400; i++)                       // 保持车体静止，读取 400 次 Z 轴样本
    {
        raw = Imu_ReadGyroZRaw();
        sum += raw;                                 // 累积原始计数，用于求静止平均值
        /* 同时统计最大、最小值，检查传感器读数是否变化。 */
        if (raw < raw_min) { raw_min = raw; }
        if (raw > raw_max) { raw_max = raw; }
        if ((i & 0x0Fu) == 0u)
        {
            BSP_WdgFeed();                          // 每 16 个样本喂狗一次
        }
        Delay_ms(5);                                // 每次读数后等待 5 ms
    }

    g_gyro_bias_raw = (float)sum / 400.0f;          // 原始计数均值作为零偏
    g_gyro_z_dps = 0.0f;
    g_yaw_deg = 0.0f;

    /* 静止读数通常仍有少量噪声。若所有原始样本完全相同，将 IMU 标记为不可用。 */
    if (raw_min == raw_max)
    {
        g_imu_ok = 0;
    }

    BSP_UartSendString("gyro bias=");
    BSP_UartSendInt((int32_t)g_gyro_bias_raw);
    BSP_UartSendString(" range=");
    BSP_UartSendInt((int32_t)(raw_max - raw_min));  // 输出校准期间的最大最小差，供串口自检
    BSP_UartSendString(g_imu_ok ? " (ok)\r\n" : " (bad)\r\n");
}

/**
  * 函    数：陀螺仪去零偏、低通滤波并积分得到相对航向
  * 参    数：dt：本次积分步长，单位 s
  * 返 回 值：无
  */
void Imu_Update(float dt)
{
    int16_t raw;
    float dps;

    if (!g_imu_ok)                                  // 未就绪则保留当前航向状态
    {
        return;
    }

    raw = Imu_ReadGyroZRaw();
    dps = ((float)raw - g_gyro_bias_raw) / GYRO_LSB_PER_DPS * GYRO_Z_SIGN;  // 去除零偏，换算成 °/s 并统一转向符号

    /* 一阶低通，使用 70% 历史滤波结果和 30% 新测值。 */
    g_gyro_z_dps = 0.70f * g_gyro_z_dps + 0.30f * dps;
    g_yaw_deg += g_gyro_z_dps * dt;                 // 角速度乘以周期并累加，得到相对航向
}

/**
  * 函    数：获取当前相对航向角
  * 参    数：无
  * 返 回 值：航向角，单位 °，左转为正，按运行累计不折返
  */
float Imu_GetYaw(void)
{
    return g_yaw_deg;
}

/**
  * 函    数：获取滤波后的 Z 轴角速度
  * 参    数：无
  * 返 回 值：Z 轴角速度，单位 °/s
  */
float Imu_GetGyroZ(void)
{
    return g_gyro_z_dps;
}

/**
  * 函    数：用指定值校正软件保存的相对航向
  * 参    数：yaw：新的航向角，单位 °
  * 返 回 值：无
  */
void Imu_SetYaw(float yaw)
{
    g_yaw_deg = yaw;                                // 直接对软件积分结果校正，不写传感器寄存器
}

/**
  * 函    数：查询初始化和校准留下的 IMU 就绪标志
  * 参    数：无
  * 返 回 值：1 为就绪，0 为未就绪
  */
uint8_t Imu_IsReady(void)
{
    return g_imu_ok;
}

/**
  * 函    数：清零软件航向角和滤波后的角速度
  * 参    数：无
  * 返 回 值：无，保留上电校准所得零偏
  */
void Imu_ResetYaw(void)
{
    g_yaw_deg = 0.0f;
    g_gyro_z_dps = 0.0f;
}
