#include "stm32f10x.h"                              // STM32F10x 外设与寄存器声明
#include "Delay.h"

/**
  * 函    数：写 MPU6050 软件 I²C 的 SCL 电平
  * 参    数：BitValue：0 拉低 SCL，1 释放 SCL
  * 返 回 值：无
  */
void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);  // PB10 连接 MPU6050 的 SCL
	Delay_us(10);                                   // 留出时钟电平保持时间
}

/**
  * 函    数：写 MPU6050 软件 I²C 的 SDA 电平
  * 参    数：BitValue：0 拉低 SDA，1 释放 SDA
  * 返 回 值：无
  */
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);  // PB11 连接 MPU6050 的 SDA
	Delay_us(10);                                   // 留出数据电平建立时间
}

/**
  * 函    数：读取 MPU6050 软件 I²C 的 SDA 电平
  * 参    数：无
  * 返 回 值：SDA 当前逻辑值，0 为低电平，1 为高电平
  */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);  // 通过输入寄存器读取实际总线电平
	Delay_us(10);
	return BitValue;
}

/**
  * 函    数：初始化 MPU6050 软件 I²C 的开漏引脚
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // 打开 GPIOB 外设时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  // 开漏方式允许主从设备共同驱动总线
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  // PB10 为 SCL，PB11 为 SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);  // 释放两根信号线，使总线进入空闲
}

/**
  * 函    数：产生 I²C 起始或重复起始条件
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);                                 // SCL 为高时拉低 SDA，产生 START
	MyI2C_W_SCL(0);                                 // 拉低 SCL，准备传输第一位
}

/**
  * 函    数：产生 I²C 停止条件并释放总线
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);                                 // 提前将 SDA 拉低，建立停止条件的初始状态
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);                                 // SCL 为高时释放 SDA，产生 STOP
}

/**
  * 函    数：按最高位优先发送一个 I²C 字节
  * 参    数：Byte：要发送的 8 位数据
  * 返 回 值：无，应答需另行读取
  */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));            // SCL 为低时把当前数据位放到 SDA
		MyI2C_W_SCL(1);                             // 升高 SCL，让从机采样 SDA
		MyI2C_W_SCL(0);                             // 拉低 SCL 后再切换下一位
	}
}

/**
  * 函    数：按最高位优先读取一个 I²C 字节
  * 参    数：无
  * 返 回 值：接收的 8 位数据，应答需另行发送
  */
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	MyI2C_W_SDA(1);                                 // 释放 SDA，由从机驱动数据
	for (i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);                             // 进入接收位的高电平采样窗口
		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}  // 按高位到低位累积读取到的 1
		MyI2C_W_SCL(0);
	}
	return Byte;
}

/**
  * 函    数：在第 9 个时钟发送 ACK 或 NACK
  * 参    数：AckBit：0 为 ACK，1 为 NACK
  * 返 回 值：无
  */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);                            // 先建立 ACK/NACK 电平，再产生应答时钟
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

/**
  * 函    数：在第 9 个时钟采样从机应答
  * 参    数：无
  * 返 回 值：0 表示从机 ACK，1 表示 NACK
  */
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);                                 // 释放 SDA，让从机拉低总线表示 ACK
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();                         // 在 SCL 高电平期间读取应答
	MyI2C_W_SCL(0);
	return AckBit;
}
