#ifndef UART_PROT
#define UART_PROT

//NODE COMMANDS

#define UART_FREQUENCY '1'
#define UART_SF '2'
#define UART_BW '3'
#define UART_SYNCWORD '4'
#define UART_POWER '5'
#define UART_PREAMBLE '8'
#define UART_CR '9'
#define UART_NODENUM 'n'
#define UART_CALL 'C'
#define UART_ACKNOWLEDGE 'A'
#define UART_READ 'R'
#define UART_SAVE 'S'
#define UART_WORKING_INTERVAL 'i'
#define UART_USELED	'L'
#define UART_TEMP	'T'
#define UART_STATUS	'?'
#define UART_DEBUG	'd'



void readByte();
void uartInit();
void uartReceiveHandler();
void sendConfig(nodeSettings_t* settingsPtr);
void uartReceiveHandler (nodeSettings_t* settingsPtr);
#endif
