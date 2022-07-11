#include "main.h"

extern flag_t flag;
extern nodeSettings_t settings;

extern uint8_t uartBuffer[UART_BUFFER_SIZE];
uint8_t len;
uint8_t data;
uint8_t tempMessage[32];
uint8_t tempPos = 0;
uint16_t currentPos = 0;

void initUart(UART_HandleTypeDef *huart, SX127X_t *myRadioHandler) {
	//HAL_UART_Receive_IT(huart, &data, 1);
	HAL_UART_Receive_DMA(huart, uartBuffer, UART_BUFFER_SIZE);
}

void checkUart() {
	if (uartBuffer[currentPos]) {
		if (uartBuffer[currentPos] == '<')
			tempPos = 0;
		else if (uartBuffer[currentPos] == '>')
			handleMessage(tempPos);
		else
			tempMessage[tempPos++] = uartBuffer[currentPos];

		uartBuffer[currentPos++] = 0;
		if (currentPos>=UART_BUFFER_SIZE) currentPos = 0;
	}

}
void handleMessage(uint8_t len) {

	uint8_t l = len - 1;
	uint8_t *ptr = tempMessage + 1;
	uint32_t tmp;

	switch (tempMessage[0]) {
	case UART_FREQUENCY:
		tmp = DecToInt(ptr, l);
		settings.realFrequency = tmp;
		break;

	case UART_SF:
		tmp = DecToInt(ptr, l);
		settings.sf = tmp;
		break;

	case UART_BW:
		tmp = DecToInt(ptr, l);
		settings.bw = tmp;
		break;

	case UART_SYNCWORD:
		tmp = HexToInt(ptr, l);
		settings.sw = tmp;
		break;

	case UART_PREAMBLE:
		tmp = DecToInt(ptr, l);
		settings.preamble = tmp;
		break;

	case UART_CR:
		tmp = DecToInt(ptr, l);
		settings.cr = tmp;
		break;

	case UART_POWER:
		tmp = DecToInt(ptr, l);
		settings.power = tmp;
		break;

	case UART_NODENUM:
		settings.nodeNum = DecToInt(ptr, l);
		break;

	case UART_WORKING_INTERVAL:
		settings.workInterval = DecToInt(ptr, l);
		break;

	case UART_USELED:
		settings.useLed = DecToInt(ptr, l);
		break;

	case UART_SAVE:
		flag.saveSettings = 1;
		break;

	case UART_READ:
		flag.readConfig = 1;
		break;

	case UART_CALL:
		printf("<ANv%lx>\n", SOFTWARE_REVISION);
		break;

	case UART_STATUS:
		flag.statusRequested = 1;
		break;

	case UART_DEBUG:
		settings.debugLevel = DecToInt(ptr, l);
		break;

	default:
		printf("Bad format!");
		Error_Handler();
		break;

	}
}

void sendConfig() {
	printf("<1%lu>\n", settings.realFrequency);
	printf("<2%u>\n", settings.sf);
	printf("<3%u>\n", settings.bw);
	printf("<4%X>\n", settings.sw);
	printf("<5%u>\n", settings.power);
	printf("<8%u>\n", settings.preamble);
	printf("<9%u>\n", settings.cr);
	printf("<n%u>\n", settings.nodeNum);
	printf("<i%lu>\n", settings.workInterval);
	printf("<L%u>\n", settings.useLed);
	printf("<d%u>\n", settings.debugLevel);
}

