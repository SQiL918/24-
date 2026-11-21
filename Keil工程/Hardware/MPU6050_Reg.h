#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

#define	MPU6050_SMPLRT_DIV		0x19                // 采样率分频
#define	MPU6050_CONFIG			0x1A                // 数字低通等公共配置
#define	MPU6050_GYRO_CONFIG		0x1B                // 陀螺仪量程和自检
#define	MPU6050_ACCEL_CONFIG	0x1C                // 加速度计量程和自检

/* 传感器结果寄存器，每个通道先高字节后低字节。 */
#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47                // 航向更新连续读取从这里开始的两个字节
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B                // 电源管理 1，包含休眠位和时钟源选择
#define	MPU6050_PWR_MGMT_2		0x6C                // 电源管理 2，包含各轴待机设置
#define	MPU6050_WHO_AM_I		0x75                // 器件标识，当前接法预期为 0x68

#endif
