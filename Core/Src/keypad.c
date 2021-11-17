#include "main.h"
#include "stm32f4xx_hal.h"
#include "keypad.h"



uint8_t urun_secme(){

	// Scan column 0 (column 0 pin is grounded, other column pins is open drain)
	    HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_4,GPIO_PIN_SET);
		HAL_Delay(100);
		// Read rows
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_1))
			return '1';


		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_2))
			return '4';

		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_3))
			return '7';



		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_4))
			return '*';

		// Scan column 1 (column 1 pin is grounded, other column pins is open drain)
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_4,GPIO_PIN_SET);
		HAL_Delay(100);
		// Read rows
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_1))
		return '2';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_2))
		return '5';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_3))
		return '8';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_4))
		return '0';

		// Scan column 2 (column 2 pin is grounded, other column pins is open drain)
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_3,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_4,GPIO_PIN_SET);
		HAL_Delay(100);
		// Read rows
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_1))
		return '3';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_2))
		return '6';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_3))
		return '9';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_4))
		return '#';

		// Scan column 3 (column 3 pin is grounded, other column pins is open drain)
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_4,GPIO_PIN_RESET);
		HAL_Delay(100);
		// Read rows
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_1))
		return 'A';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_2))
		return 'B';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_3))
		return 'C';
		if (!HAL_GPIO_ReadPin(SATIR_PORT, SATIR_4))
		return 'D';

		return KEYPAD_NO_PRESSED;


}

void reset(){

	 HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_1 ,GPIO_PIN_RESET );
	 HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_2 ,GPIO_PIN_RESET );
	 HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_3 ,GPIO_PIN_RESET );
	 HAL_GPIO_WritePin(SUTUN_PORT, SUTUN_4 ,GPIO_PIN_RESET );
	 HAL_GPIO_WritePin(SATIR_PORT, SATIR_1 ,GPIO_PIN_RESET );
	 HAL_GPIO_WritePin(SATIR_PORT, SATIR_2 ,GPIO_PIN_RESET );
	 HAL_GPIO_WritePin(SATIR_PORT, SATIR_3 ,GPIO_PIN_RESET );
	 HAL_GPIO_WritePin(SATIR_PORT, SATIR_4 ,GPIO_PIN_RESET );

}
