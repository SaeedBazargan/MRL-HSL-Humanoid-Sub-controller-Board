#ifndef sbzrgnCM_H_
#define sbzrgnCM_H_

#include "stm32f4xx_hal.h"

/*----------------------------------------------------------------------------*/
#define MY_ID		      			0xC8	//CMBoard ID
#define DXL_Error			   		0	
#define DXL_READ_DATA       0x02
#define DXL_WRITE_DATA      0x03
#define BUFFER_SIZE  				1000
#define MAX_PACKET_SIZE 		250
/*----------------------------------------------------------------------------*/
typedef enum
{
	SB_UART_ERROR    = 0x00U,
	SB_IMU_ERROR     = 0x01U,
	SB_BATTERY_ERROR = 0x02U
}SB_ErrorTypeDef;
/*----------------------------------------------------------------------------*/
typedef enum
{
	SB_OK    = 0x00U,
	SB_ERROR = 0x01U,
	SB_BUSY  = 0x02U	
}SB_StatusTypeDef;
/*----------------------------------------------------------------------------*/
typedef enum 
{
	SB_LOCKED   = 0x00U,
	SB_UNLOCKED = 0x01U
}SB_LockTypeDef;
/*----------------------------------------------------------------------------*/
typedef struct
{
	uint8_t Length;
	uint8_t CurrentState;
	uint8_t InputData;
	uint16_t BufferCurrentIndex;
	uint8_t Buffer[300];
}UARTLineTypeDef;
/*----------------------------------------------------------------------------*/
typedef struct
{
	uint8_t InputData;
	uint8_t Length;
	uint8_t CurrentState;
	uint16_t BufferCurrentIndex;
	uint16_t BufferLastIndex;
	uint8_t Buffer[BUFFER_SIZE];
}USBLineTypeDef;
/*----------------------------------------------------------------------------*/
#endif /* __MRL_HSL_SB_H__ */
