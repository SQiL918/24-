#include "stm32f10x.h"                              // STM32F10x 外设与寄存器声明
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0                    // 7 位从机地址 0x68 左移一位，最低位 0 表示写

/**
  * 函    数：通过软件 I²C 写一个 MPU6050 寄存器
  * 参    数：RegAddress：目标寄存器地址
  *           Data：要写入的 8 位值
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();                                  // 开始写寄存器事务
	MyI2C_SendByte(MPU6050_ADDRESS);                // 发送从机写地址
	MyI2C_ReceiveAck();                             // 采样从机应答，此接口不向调用方返回应答状态
	MyI2C_SendByte(RegAddress);                     // 发送寄存器地址，设置内部地址指针
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Data);                           // 写入寄存器数据
	MyI2C_ReceiveAck();
	MyI2C_Stop();                                   // 结束事务，释放总线
}

/**
  * 函    数：通过软件 I²C 读取一个 MPU6050 寄存器
  * 参    数：RegAddress：目标寄存器地址
  * 返 回 值：读到的 8 位寄存器内容
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);                // 先以写方向设置要读取的寄存器地址
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();                                  // 不释放总线，产生重复 START
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);         // 将最低位设为 1，切换到读方向
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();                     // 读取目标寄存器字节
	MyI2C_SendAck(1);                               // 发送 NACK，通知从机本次只读一个字节
	MyI2C_Stop();
	
	return Data;
}

/**
  * 函    数：按基础驱动的默认参数配置 MPU6050
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
	/* 这是基础驱动的配置入口。小车实际控制使用 App/imu.c 的 Imu_Init。 */
	MyI2C_Init();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);     // 退出休眠并选择 X 轴陀螺仪 PLL 作时钟源
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);     // 使能所有加速度和陀螺仪轴
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);     // 采样分频设为 1 + 9
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);         // 数字低通滤波寄存器配置为档位 6
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);    // 陀螺仪满量程设为 ±2000 °/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);   // 加速度计满量程设为 ±16 g
}

/**
  * 函    数：读取 MPU6050 的 WHO_AM_I 标识寄存器
  * 参    数：无
  * 返 回 值：设备标识字节，本板预期为 0x68
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
  * 函    数：逐寄存器读取三轴加速度和三轴角速度
  * 参    数：AccX：写入 X 轴加速度原始值的指针
  *           AccY：写入 Y 轴加速度原始值的指针
  *           AccZ：写入 Z 轴加速度原始值的指针
  *           GyroX：写入 X 轴陀螺仪原始值的指针
  *           GyroY：写入 Y 轴陀螺仪原始值的指针
  *           GyroZ：写入 Z 轴陀螺仪原始值的指针
  * 返 回 值：无，六路结果均为有符号 16 位原始计数
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;                   // 高低字节合并为 X 轴加速度原始值
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;                   // 保存 Y 轴加速度原始值
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;                   // 保存 Z 轴加速度原始值
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;                  // 保存 X 轴角速度原始值
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;                  // 保存 Y 轴角速度原始值
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;                  // 保存 Z 轴角速度原始值
}
