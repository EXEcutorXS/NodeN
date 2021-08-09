#ifndef UART_PROT
#define UART_PROT

#include "main.h"

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

uint32_t DecToInt(uint8_t * string, uint8_t len);
uint32_t HexToInt(uint8_t * string, uint8_t len);
uint32_t _pow10(uint8_t value);
uint32_t pow16(uint8_t value);

void readByte();
void uartInit();
void uartReceiveHandler();
void sendConfig(void);
#endif
