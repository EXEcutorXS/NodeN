#include "main.h"

extern flag_t flag;

SX127X_t* myRadio;

uint8_t uartRx[32];
uint8_t uartPos;
uint8_t len;
uint8_t data;

void initUart(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma, SX127X_t* myRadioHandler)
{
	HAL_UART_Receive_DMA(huart, &data, 1);
	hdma->XferCpltCallback = readByte;
}

void readByte (UART_HandleTypeDef* huart)
{
	if (data == '<')
		uartPos = 0;
	else if (data == '>')
		{
			len = uartPos;
			flag.uartRx = 1;
		}
	else
		uartRx[uartPos++] = data;
}

void uartReceiveHandler (nodeSettings_t* settingsPtr)
{

	uint8_t l = len - 1;
	uint8_t *ptr = uartRx + 1;
	uint32_t tmp;

	switch (uartRx[0])
	{
		case UART_FREQUENCY:
			tmp = DecToInt (ptr, l);
			settingsPtr->realFrequency = tmp;
			break;

		case UART_SF:
			tmp = DecToInt (ptr, l);
			settingsPtr->sf = tmp;
			break;

		case UART_BW:
			tmp = DecToInt (ptr, l);
			settingsPtr->bw = tmp;
			break;

		case UART_SYNCWORD:
			tmp = HexToInt (ptr, l);
			settingsPtr->sw = tmp;
			break;

		case UART_PREAMBLE:
			tmp = DecToInt (ptr, l);
			settingsPtr->preamble = tmp;
			break;

		case UART_CR:
			tmp = DecToInt (ptr, l);
			settingsPtr->cr = tmp;
			break;

		case UART_POWER:
			tmp = DecToInt (ptr, l);
			settingsPtr->power = tmp;
			break;

		case UART_NODENUM:
			settingsPtr->nodeNum = DecToInt (ptr, l);
			break;

		case UART_WORKING_INTERVAL:
			settingsPtr->workInterval = DecToInt (ptr, l);
			break;

		case UART_USELED:
			settingsPtr->useLed = DecToInt (ptr, l);
			break;

		case UART_SAVE:
			flag.saveSettings = 1;
			break;

		case UART_READ:
			flag.readConfig = 1;
			break;

		case UART_CALL:
			printf ("<ANv%lx>", SOFTWARE_REVISION);
			break;

		case UART_STATUS:
			flag.statusRequested = 1;
			break;

		default:
			printf ("Bad format!");
			Error_Handler ();
			break;

	}

}

void sendConfig (nodeSettings_t* settingsPtr)
{
	printf ("<1%lu>", settingsPtr->realFrequency);
	printf ("<2%u>", settingsPtr->sf);
	printf ("<3%u>", settingsPtr->bw);
	printf ("<4%X>", settingsPtr->sw);
	printf ("<5%u>", settingsPtr->power);
	printf ("<8%u>", settingsPtr->preamble);
	printf ("<9%u>", settingsPtr->cr);
	printf ("<n%u>", settingsPtr->nodeNum);
	printf ("<i%lu>", settingsPtr->workInterval);
	printf ("<L%u>\n", settingsPtr->useLed);
}

