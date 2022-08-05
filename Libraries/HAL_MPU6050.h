#ifndef __HAL_MPU6050_H__
#define __HAL_MPU6050_H__

#include "stdint.h"
#include "stm32f4xx_hal.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR							0xD0
/* Who I am register value */
#define MPU6050_I_AM									0x68
#define MPU6050_WHO_AM_I							0x75
/* MPU6050 registers */
#define MPU6050_CONFIG								0x1A
#define MPU6050_GYRO_CONFIG						0x1B
#define MPU6050_ACCEL_CONFIG					0x1C
#define MPU6050_ACCEL_XOUT_H					0x3B
#define MPU6050_ACCEL_XOUT_L					0x3C
#define MPU6050_ACCEL_YOUT_H					0x3D
#define MPU6050_ACCEL_YOUT_L					0x3E
#define MPU6050_ACCEL_ZOUT_H					0x3F
#define MPU6050_ACCEL_ZOUT_L					0x40
#define MPU6050_TEMP_OUT_H						0x41
#define MPU6050_TEMP_OUT_L						0x42
#define MPU6050_GYRO_XOUT_H						0x43
#define MPU6050_GYRO_XOUT_L						0x44
#define MPU6050_GYRO_YOUT_H						0x45
#define MPU6050_GYRO_YOUT_L						0x46
#define MPU6050_GYRO_ZOUT_H						0x47
#define MPU6050_GYRO_ZOUT_L						0x48
#define MPU6050_PWR_MGMT_1						0x6B
#define MPU6050_PWR_MGMT_2						0x6C

typedef enum  {
	MPU6050_RESULT_OK    = 0x00U,
	MPU6050_RESULT_ERROR = 0X01U
}MPU6050_Result;

/* Digital Low Pass Filter */
typedef enum
{
	MPU6050_DLPF_ACCEL_260_GYRO256 = 0x00,
	MPU6050_DLPF_ACCEL_184_GYRO188 = 0x01,
	MPU6050_DLPF_ACCEL_94_GYRO98   = 0x02,
	MPU6050_DLPF_ACCEL_44_GYRO42   = 0x03,
	MPU6050_DLPF_ACCEL_21_GYRO20   = 0x04,
	MPU6050_DLPF_ACCEL_10_GYRO10   = 0x05,
	MPU6050_DLPF_ACCEL_5_GYRO5     = 0x06,
}MPU6050_DLPFTypeDef;

/* Parameters for accelerometer range */
typedef enum  {
	MPU6050_Accelerometer_2  = 0x00, /*!< Range is +- 2G */
	MPU6050_Accelerometer_4  = 0x01, /*!< Range is +- 4G */
	MPU6050_Accelerometer_8  = 0x02, /*!< Range is +- 8G */
	MPU6050_Accelerometer_16 = 0x03  /*!< Range is +- 16G */
}MPU6050_AccelTypeDef;

/* Parameters for gyroscope range */
typedef enum {
	MPU6050_Gyroscope_250  = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU6050_Gyroscope_500  = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU6050_Gyroscope_1000 = 0x02,  /*!< Range is +- 1000 degrees/s */
	MPU6050_Gyroscope_2000 = 0x03   /*!< Range is +- 2000 degrees/s */
}MPU6050_GyroTypeDef;

/* Temprature Register */
typedef enum
{
	MPU6050_TEMP_ON  = 0x00U,
	MPU6050_TEMP_OFF = 0x08U
}MPU6050_TempTypeDef;

/* Main MPU6050 structure */
typedef struct  {
	
	uint8_t Address;                       /*!< I2C address of device. */
	MPU6050_TempTypeDef Temp;
	MPU6050_DLPFTypeDef DLPF;
	MPU6050_GyroTypeDef GyroRange;
	MPU6050_AccelTypeDef AccelRange;
	
}MPU6050TypeDef;

MPU6050_Result MPU6050_Init(I2C_HandleTypeDef* I2Cx , MPU6050TypeDef* datastruct);

MPU6050_Result MPU6050_ReadData(I2C_HandleTypeDef* I2Cx, MPU6050TypeDef* datastruct, uint8_t* buffer, uint8_t addr, uint8_t num);

void MPU6050_Error_Handler(void);

#endif /* __HAL_MPU6050_H__*/
