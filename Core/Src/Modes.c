#include "main.h"

extern SX127X_t myRadio;

void ReceivingTest() {
	uint32_t recTime = 0x80000000;
	uint32_t recTime1 = 0x80000000;
	uint32_t recTime2 = 0x80000000;
	bool repeaterMode = false;
	debugLogTime("Receiving test activated");
	while (1) {

		SX127X_Handler(&myRadio);
		if (myRadio.readBytes > 0) {
			if (myRadio.badCrc == 0) {
				if (myRadio.rxBuf[0] == 255 && myRadio.rxBuf[1] == 255) {
					repeaterMode = true;
					if (myRadio.rxBuf[2] == 1)
						recTime1 = HAL_GetTick();
					if (myRadio.rxBuf[2] == 2)
						recTime2 = HAL_GetTick();
				} else {
					recTime = HAL_GetTick();
				}

			}

			myRadio.readBytes = 0;
		}
		if (repeaterMode) {
			if (myRadio.signalDetected) {
				HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_ON);
				HAL_GPIO_WritePin(BLUE_GPIO_Port, ORANGE_Pin, LED_ON);
			} else {
				HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_OFF);
				HAL_GPIO_WritePin(BLUE_GPIO_Port, ORANGE_Pin, LED_OFF);
			}

			if (HAL_GetTick() - recTime1 < 300)
				HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_ON);
			else
				HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_OFF);
			if (HAL_GetTick() - recTime2 < 300)
				HAL_GPIO_WritePin(ORANGE_GPIO_Port, BLUE_Pin, LED_ON);
			else
				HAL_GPIO_WritePin(ORANGE_GPIO_Port, BLUE_Pin, LED_OFF);
		} else {
			if (myRadio.signalDetected)
				HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_ON);
			else
				HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_OFF);
			if (HAL_GetTick() - recTime < 300)
				HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_ON);
			else
				HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_OFF);

		}

	}
}

/**
 * @brief Infinite cycle with Ping Test
 * Node sends status every 2 seconds
 * Green light means valid response from Base station
 * Red light means transmission in progress
 * @param None
 * @retval None
 */
void PingTest() {

	debugLogTime("Ping test activated");
	while (1) {
		static uint32_t lastTrans;
		static uint32_t recTime;

		SX127X_Handler(&myRadio);

		if (myRadio.readBytes > 0) {
			if (myRadio.badCrc == 0)
				recTime = HAL_GetTick();
			myRadio.readBytes = 0;
		}

		if (HAL_GetTick() - lastTrans > 2000) {
			lastTrans = HAL_GetTick();
		}
		if (HAL_GetTick() - recTime < 300)
			HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_ON);
		else
			HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_OFF);

		if (myRadio.status == TX)
			HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_ON);
		else
			HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_OFF);
	}
}
