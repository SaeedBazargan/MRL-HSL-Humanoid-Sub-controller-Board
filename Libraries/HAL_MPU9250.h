#ifndef __HAL_MPU9250_H__
#define __HAL_MPU9250_H__

#include "stm32f4xx_hal.h"

/* Default I2C address */
#define MPU9250_I2C_ADDR							0xD0
/* Who I am register value */
#define MPU9250_I_AM									0x71
#define MPU9250_WHO_AM_I							0x75
/* MPU9250 registers */
#define MPU9250_CONFIG								0x1A
#define MPU9250_GYRO_CONFIG						0x1B
#define MPU9250_ACCEL_CONFIG					0x1C
#define MPU9250_ACCEL_CONFIG_2        0x1D
#define MPU9250_ACCEL_XOUT_H					0x3B
#define MPU9250_ACCEL_XOUT_L					0x3C
#define MPU9250_ACCEL_YOUT_H					0x3D
#define MPU9250_ACCEL_YOUT_L					0x3E
#define MPU9250_ACCEL_ZOUT_H					0x3F
#define MPU9250_ACCEL_ZOUT_L					0x40
#define MPU9250_TEMP_OUT_H						0x41
#define MPU9250_TEMP_OUT_L						0x42
#define MPU9250_GYRO_XOUT_H						0x43
#define MPU9250_GYRO_XOUT_L						0x44
#define MPU9250_GYRO_YOUT_H						0x45
#define MPU9250_GYRO_YOUT_L						0x46
#define MPU9250_GYRO_ZOUT_H						0x47
#define MPU9250_GYRO_ZOUT_L						0x48
#define MPU9250_USER_CTRL             0x6A
#define MPU9250_PWR_MGMT_1						0x6B

typedef enum
{
	MPU9250_RESULT_OK    = 0x00U,
	MPU9250_RESULT_ERROR = 0X01U
}MPU9250_Result;

/* Parameters for accelerometer range */
typedef enum
{
	MPU9250_Accelerometer_2  = 0x00, /*!< Range is +- 2G */
	MPU9250_Accelerometer_4  = 0x01, /*!< Range is +- 4G */
	MPU9250_Accelerometer_8  = 0x02, /*!< Range is +- 8G */
	MPU9250_Accelerometer_16 = 0x03  /*!< Range is +- 16G */
}MPU9250_AccelTypeDef;

/* Parameters for gyroscope range */
typedef enum
{
	MPU9250_Gyroscope_250  = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU9250_Gyroscope_500  = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU9250_Gyroscope_1000 = 0x02,  /*!< Range is +- 1000 degrees/s */
	MPU9250_Gyroscope_2000 = 0x03   /*!< Range is +- 2000 degrees/s */
}MPU9250_GyroTypeDef;

/*Gyroscope Digital Low Pass Filter --BW(bandwidth (Hz)) --D(delay (uS))*/
typedef enum
{
	BW250_D970    = 0x00,
	BW184_D2900   = 0x01,
	BW92_D3900    = 0x02,
	BW41_D5900    = 0x03,
	BW20_D9900    = 0x04,
	BW10_D17850   = 0x05,
	BW5_D33480    = 0x06,
	BW3600_D170   = 0x07
}MPU9250_GYRO_DLPFTypeDef;

/*Accelerometer Digital Low Pass Filter --BW(bandwidth (Hz)) --D(delay (uS))*/
typedef enum
{
	BW218_D1880   = 0x00,
	BW99_D2880    = 0x02,
	BW44_D4880    = 0x03,
	BW21_D8870    = 0x04,
	BW10_D16830   = 0x05,
	BW5_D32480    = 0x06,
	BW420_D1380   = 0x07
}MPU9250_ACCEL_DLPFTypeDef;

/* Main MPU9250 structure */
typedef struct
{
	MPU9250_GYRO_DLPFTypeDef    Gyro_DLPF;
	MPU9250_ACCEL_DLPFTypeDef   Accel_DLPF;
	MPU9250_GyroTypeDef         Gyro_Range;
	MPU9250_AccelTypeDef        Accel_Range;
}MPU9250TypeDef;

MPU9250_Result MPU9250_Init(SPI_HandleTypeDef* SPIx , MPU9250TypeDef* datastruct);
MPU9250_Result MPU9250_ReadData(SPI_HandleTypeDef* SPIx, uint8_t* buffer, uint8_t addr, uint8_t num);
MPU9250_Result MPU9250_WriteData(SPI_HandleTypeDef* SPIx, uint8_t addr, uint8_t data);

#endif /* __HAL_MPU9250_H__*/
