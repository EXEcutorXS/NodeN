#include "main.h"
extern flag_t flag;
extern nodeSettings_t settings;
extern UART_HandleTypeDef huart1;
extern SX127X_t myRadio;
extern uint8_t uartIn;
extern uint8_t configCrc;
extern uint8_t receivedCrc;
extern uint32_t version;
extern int8_t calibratedTemp;
uint8_t uartRx[32];
uint8_t uartIn;
uint8_t uartPos;
uint8_t len;

void readByte ()
{
	if (uartIn == '<')
		uartPos = 0;
	else if (uartIn == '>')
		{
			len = uartPos;
			flag.uartRx = 1;
		}
	else
		uartRx[uartPos++] = uartIn;
}

void uartReceiveHandler ()
{

	uint8_t l = len - 1;
	uint8_t *ptr = uartRx + 1;
	uint32_t tmp;

	switch (uartRx[0])
	{
		case UART_FREQUENCY:
			tmp = DecToInt (ptr, l);
			settings.realFrequency = tmp;
			break;

		case UART_SF:
			tmp = DecToInt (ptr, l);
			settings.sf = tmp;
			break;

		case UART_BW:
			tmp = DecToInt (ptr, l);
			settings.bw = tmp;
			break;

		case UART_SYNCWORD:
			tmp = HexToInt (ptr, l);
			settings.sw = tmp;
			break;

		case UART_PREAMBLE:
			tmp = DecToInt (ptr, l);
			settings.preamble = tmp;
			break;

		case UART_CR:
			tmp = DecToInt (ptr, l);
			settings.cr = tmp;
			break;

		case UART_POWER:
			tmp = DecToInt (ptr, l);
			settings.power = tmp;
			break;

		case UART_NODENUM:
			settings.nodeNum = DecToInt (ptr, l);
			break;

		case UART_WORKING_INTERVAL:
			settings.workInterval = DecToInt (ptr, l);
			break;

		case UART_USELED:
			settings.useLed = DecToInt (ptr, l);
			break;

		case UART_SAVE:
			flag.saveSettings = 1;
			break;

		case UART_READ:
			flag.readConfig = 1;
			break;

		case UART_CALL:
			printf ("<ANv%lx>", version);
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

void sendConfig (void)
{
	printf ("<1%lu>", settings.realFrequency);
	printf ("<2%u>", settings.sf);
	printf ("<3%u>", settings.bw);
	printf ("<4%X>", settings.sw);
	printf ("<5%u>", settings.power);
	printf ("<8%u>", myRadio.preamble);
	printf ("<9%u>", settings.cr);
	printf ("<n%u>", settings.nodeNum);
	printf ("<i%lu>", settings.workInterval);
	printf ("<L%u>\n", settings.useLed);

}

