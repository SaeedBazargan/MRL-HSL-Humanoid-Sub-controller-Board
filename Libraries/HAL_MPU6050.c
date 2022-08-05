#include "HAL_MPU6050.h"

MPU6050_Result MPU6050_Init(I2C_HandleTypeDef* I2Cx , MPU6050TypeDef* datastruct)
{
	uint8_t WHO_AM_I = (uint8_t)MPU6050_WHO_AM_I;
	uint8_t temp;
	uint8_t transmit_buffer[2]={0,0};

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(I2Cx, datastruct->Address, 2, 5) != HAL_OK)
	{
		return MPU6050_RESULT_ERROR;
	}
	
	/* Check who am I */
	HAL_I2C_Master_Transmit(I2Cx, datastruct->Address, &WHO_AM_I, 1, 1);
	
	HAL_I2C_Master_Receive(I2Cx, datastruct->Address, &temp, 1, 1);
	
	if(temp != MPU6050_I_AM)
	{
		return MPU6050_RESULT_ERROR;
	}
	
	/* Reset all the Registers */
	transmit_buffer[0] = MPU6050_PWR_MGMT_1;
	transmit_buffer[1] = 0x80;	
	HAL_I2C_Master_Transmit(I2Cx, datastruct->Address, transmit_buffer, 2, 10);
	
	HAL_Delay(5);
	
	/* Wakeup MPU6050 */
	transmit_buffer[0] = MPU6050_PWR_MGMT_1;
	transmit_buffer[1] = datastruct->Temp;
	HAL_I2C_Master_Transmit(I2Cx, datastruct->Address, transmit_buffer, 2, 10);

	/* Configure Config Register */
	transmit_buffer[0] = MPU6050_CONFIG;
	transmit_buffer[1] = datastruct->DLPF;
	HAL_I2C_Master_Transmit(I2Cx, datastruct->Address, transmit_buffer, 2, 10);
	
	/* Configure Gyro_Config Register */
	transmit_buffer[0] = MPU6050_GYRO_CONFIG;
	transmit_buffer[1] = (datastruct->GyroRange) << 3;
	HAL_I2C_Master_Transmit(I2Cx, datastruct->Address, transmit_buffer, 2, 10);

	/* Configure Accel_Config Register */
	transmit_buffer[0] = MPU6050_ACCEL_CONFIG;
	transmit_buffer[1] = (datastruct->AccelRange) << 3;	
	HAL_I2C_Master_Transmit(I2Cx, datastruct->Address, transmit_buffer, 2, 10);
	
	/* Return OK */
	return MPU6050_RESULT_OK;
}

MPU6050_Result MPU6050_ReadData(I2C_HandleTypeDef* I2Cx, MPU6050TypeDef* datastruct, uint8_t* buffer, uint8_t addr, uint8_t num)
{
	uint8_t reg = addr;
	HAL_I2C_Master_Transmit(I2Cx, datastruct->Address, &reg, 1, 10);
	
	HAL_I2C_Master_Receive(I2Cx, datastruct->Address, buffer, num, 10);

	/* Return OK */
	return MPU6050_RESULT_OK;
}

void MPU6050_Error_Handler(void)
{
	HAL_Delay(2);
}
