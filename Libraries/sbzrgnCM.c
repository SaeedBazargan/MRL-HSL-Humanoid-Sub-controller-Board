#include "sbzrgnCM.h"

void SB_Error_Handler(SB_ErrorTypeDef sb_error)
{
	switch(sb_error)
	{
		case SB_UART_ERROR:
//			HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
//				HAL_Delay(3000);
//			HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
//				HAL_Delay(3000);
		break;
		
		case SB_IMU_ERROR:
//			HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
//				HAL_Delay(1000);
//			HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
//				HAL_Delay(1000);
		break;
		
		case SB_BATTERY_ERROR:
//			HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_SET);
//				HAL_Delay(2000);
//			HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, GPIO_PIN_RESET);
//				HAL_Delay(2000);
		break;
	}
}
