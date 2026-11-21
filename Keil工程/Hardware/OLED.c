#include "stm32f10x.h"
#include "OLED_Font.h"

/*引脚配置*/
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))  // PB8 为 OLED 时钟线
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))  // PB9 为 OLED 数据线

/*引脚初始化*/
/**
  * 函    数：初始化 OLED 独立软件 I²C 的引脚
  * 参    数：无
  * 返 回 值：无
  */
void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // 打开 GPIOB 时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  // 开漏输出，置 1 时释放总线
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;       // 先配置 SCL 引脚
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;       // 以同样的方式配置 SDA
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);                                  // 两根总线线均释放至空闲高电平
}

/**
  * 函    数：产生 OLED 软件 I²C 起始条件
  * 参    数：无
  * 返 回 值：无
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);                                  // 在 SCL 为高时拉低 SDA，产生 START
	OLED_W_SCL(0);
}

/**
  * 函    数：产生 OLED 软件 I²C 停止条件
  * 参    数：无
  * 返 回 值：无
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);                                  // 在 SCL 为高时释放 SDA，产生 STOP
}

/**
  * 函    数：发送一个 OLED 总线字节并提供应答时钟
  * 参    数：Byte：要发送的 8 位数据，最高位先发
  * 返 回 值：无，不读取应答
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));             // 按位掩码取出当前发送位
		OLED_W_SCL(1);                              // 产生时钟上沿，让 OLED 采样 SDA
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
}

/**
  * 函    数：向 SSD1306 写入一个命令字节
  * 参    数：Command：命令或其参数字节
  * 返 回 值：无
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command);                     // 发送命令或命令的后续参数
	OLED_I2C_Stop();
}

/**
  * 函    数：向 SSD1306 显存写入一个数据字节
  * 参    数：Data：页内单列的 8 位点阵数据
  * 返 回 值：无
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);                        // 写入一列 8 像素的显存数据
	OLED_I2C_Stop();
}

/**
  * 函    数：设置 SSD1306 页寻址模式下的写入位置
  * 参    数：Y：页号，0～7，每页高 8 像素
  *           X：像素列号，0～127
  * 返 回 值：无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/**
  * 函    数：清空 OLED 的 8 页显存
  * 参    数：无
  * 返 回 值：无
  */
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)                         // 遍历显示区域的全部 8 页
	{
		OLED_SetCursor(j, 0);                       // 从每一页的最左列开始写
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);                   // 写零覆盖原来的点阵
		}
	}
}

/**
  * 函    数：显示一个 16×16 汉字点阵
  * 参    数：X：起始像素列，0～111
  *           Page：起始页，0～6
  *           Bitmap：32 字节点阵，前后各 16 字节对应上、下半部分
  * 返 回 值：无
  */
void OLED_ShowChinese16(uint8_t X, uint8_t Page, const uint8_t *Bitmap)
{
	uint8_t i;
	OLED_SetCursor(Page, X);                        // 光标定位到字模上半部分
	for (i = 0; i < 16; i++)
	{
		OLED_WriteData(Bitmap[i]);                  // 顺序写入上半部分的 16 列
	}
	OLED_SetCursor(Page + 1, X);                    // 同列跳到下一页，接续字模下半部分
	for (i = 0; i < 16; i++)
	{
		OLED_WriteData(Bitmap[i + 16]);             // 顺序写入下半部分的 16 列
	}
}

/**
  * 函    数：显示一个 8×16 ASCII 字符
  * 参    数：Line：字符行，1～4
  *           Column：字符列，1～16
  *           Char：字库支持的可见 ASCII 字符
  * 返 回 值：无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

/**
  * 函    数：从指定位置逐字符显示字符串
  * 参    数：Line：起始字符行，1～4
  *           Column：起始字符列，1～16
  *           String：以空字符结尾的可见 ASCII 字符串，长度须适合本行余量
  * 返 回 值：无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);  // 按字符推进列位置
	}
}

/**
  * 函    数：计算固定宽度数字显示所需的整数幂
  * 参    数：X：底数
  *           Y：非负整数指数
  * 返 回 值：X 的 Y 次幂，按 uint32_t 表示
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;                                // 累乘底数，得到整数幂
	}
	return Result;
}

/**
  * 函    数：以固定宽度显示无符号十进制数
  * 参    数：Line：字符行，1～4
  *           Column：起始字符列，1～16
  *           Number：待显示的 uint32_t 值
  *           Length：数字位数，1～10
  * 返 回 值：无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		/* 除去低位，再取模提取当前数字，从最高显示位开始写。 */
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * 函    数：在首列显示符号，再显示定宽十进制数字
  * 参    数：Line：字符行，1～4
  *           Column：符号所在字符列
  *           Number：待显示的有符号数，负值的绝对值须可用 int32_t 表示
  *           Length：数字位数，1～10，不含符号
  * 返 回 值：无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');           // 在数字之前占一列显示正号
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');           // 在数字之前占一列显示负号
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		/* 右移一列，为符号留出空间，再逐位显示绝对值。 */
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * 函    数：以固定宽度显示大写十六进制数
  * 参    数：Line：字符行，1～4
  *           Column：起始字符列，1～16
  *           Number：待显示的 uint32_t 值
  *           Length：十六进制位数，1～8
  * 返 回 值：无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;  // 提取当前十六进制位
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');  // 把 10～15 映射到字符 A～F
		}
	}
}

/**
  * 函    数：以固定宽度显示二进制数
  * 参    数：Line：字符行，1～4
  *           Column：起始字符列，1～16
  *           Number：待显示的无符号值
  *           Length：本屏建议显示的位数，1～16
  * 返 回 值：无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		/* 按二进制权值提取对应位，显示字符 0 或 1。 */
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * 函    数：初始化 OLED 控制器并清屏
  * 参    数：无
  * 返 回 值：无
  */
void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			//端口初始化
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);                        // 选择默认显示振荡参数
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);                        // 复用行数设为 64
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);                        // 显示偏移为 0
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);                        // 选择当前屏幕使用的 COM 引脚布局
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);                        // 对比度值设为 0xCF

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);                        // 写入两个预充电阶段的周期参数

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);                        // 设置 VCOMH 取消选择电平

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);                        // 使能片内充电泵

	OLED_WriteCommand(0xAF);	//开启显示
		
	OLED_Clear();				//OLED清屏
}
