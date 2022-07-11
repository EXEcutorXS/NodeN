/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef DEBUG
const char *statuses[] = { "UNINITIALISED", "SLEEP", "STANDBY", "TX", "RX" };
#endif

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

LPTIM_HandleTypeDef hlptim1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
flag_t flag = { 0, };
uint8_t lpTimWdCnt = 0;

SX127X_t myRadio = { 0, };

uint16_t recomendedDelay = 600;

uint8_t regs[32];

uint32_t poweredChangeMoment = 0;
float temp;

nodeStatus_t status = { 0, };
nodeSettings_t settings = { 0, };

downlinkMessage_t *rxMes = (downlinkMessage_t*) myRadio.rxBuf;

uint32_t lastTransTime = 0;
uint32_t interval = DEFAULT_INTERVAL;


uint8_t uartBuffer[UART_BUFFER_SIZE];
uint16_t uartWritePos = 0;
uint16_t uartReadPos = 0;

uint8_t triesToSend = 0;

bool dontSleep = false;
bool wfa = false;

uint32_t totalMessages = 0;
uint32_t acknowledgeMiss = 0;

uplinkMessage_t *txMes = (uplinkMessage_t*) myRadio.txBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_WWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int fd, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, 1000L);
	return len;
}

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 1000);
	return ch;
}

void debugLogTime(char *string) {
	if (settings.debugLevel) {
		uint32_t time = RTC->TR;
		printf("\n");
		printf("%08lu", HAL_GetTick());
		printf(" %02x:%02x:%02x ", (uint16_t) (time >> 16) & 0xFF,
				(uint16_t) (time >> 8) & 0xFF, (uint16_t) time & 0xFF);
		printf(string);
		printf("\n");
	}
}

void debugLog(char *string) {
	if (settings.debugLevel) {
		printf(string);
		printf("\n");
	}
}

void debugLogInt(char *format, int n) {
	if (settings.debugLevel) {
		printf(format, n);
		printf("\n");
	}
}

void debugLogString(char *format, char *string) {
	if (settings.debugLevel) {
		printf(format, string);
		printf("\n");
	}
}

/**
 * @brief Gets voltage level of MCU Vdd pin
 * Calculates MCU Vdd voltage by internal reference
 * @param None
 * @retval  Volage
 */
float getVoltage() {
	uint16_t adc[2];
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc, 2);
	HAL_Delay(5);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc, 2);
	HAL_Delay(2);
	return 3.0f * (float) VREF_CAL_VALUE / (float) adc[1];
}

/**
 * @brief Gets Temperature of Node
 * Can be calculated through NTC thermistor or TI IC
 * @param None
 * @retval  Temperature in Celsius
 */
float getTemperature() {
	uint16_t adc;
	float Rt = 0;
	float tKelvin = 0;
	float tCelsius = 0;
#ifdef USE_NTC
	HAL_GPIO_WritePin(TempPower_GPIO_Port, TempPower_Pin, 1);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) &adc, 1);
	HAL_Delay(5);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) &adc, 1);
	HAL_Delay(2);
	Rt = R_BALANCE * (4096.0 / (float) adc - 1.0F);
	tKelvin = (BETA * HOME_TEMP)
			/ (BETA + (HOME_TEMP * log(Rt / R_THERMISTOR_DEFAULT)));

	tCelsius = tKelvin - 273.15;
	HAL_GPIO_WritePin(TempPower_GPIO_Port, TempPower_Pin, 0);
#endif
	return tCelsius;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	debugLogTime("RTC Interrupt");
	lpTimWdCnt = 0;
	flag.rtcAlarm = 1;
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim) {
	if (lpTimWdCnt > (settings.workInterval * 2) / WATCHDOG_INTERVAL) {
		debugLogTime("###LP WatchDog activated! Reset MCU!");
		SCB->AIRCR |= SCB_AIRCR_SYSRESETREQ_Msk;
	} else {
		debugLogTime("LP Interrupt");
		debugLogInt("LP Watchdog cnt is %d", ++lpTimWdCnt);
	}
}

/**
 * @brief Initialises radio handler structure
 * @param None
 * @retval  None
 */
void initiateSettings() {
	myRadio.sf = settings.sf;
	myRadio.bw = settings.bw;
	myRadio.cr = settings.cr;
	myRadio.syncWord = settings.sw;
	myRadio.frequency = settings.realFrequency / 61.035f;
	myRadio.power = settings.power;
	myRadio.preamble = settings.preamble;
}

/**
 * @brief Initializes default node settings
 * @param None
 * @retval None
 */
void defaultSettings() {
	settings.nodeNum = 0;
	settings.workInterval = 600;
	settings.voltageTreshold = 2.0f;
	settings.bw = SX127X_LORA_BW_125KHZ;
	settings.cr = SX127X_CR_4_8;
	settings.sf = SX127X_LORA_SF_12;
	settings.sw = 0x1;
	settings.power = SX127X_POWER_20DBM;
	settings.realFrequency = DEF_FREQUENCY;
	settings.preamble = 5;
	settings.useLed = true;
	settings.debugLevel = 0;
	initiateSettings();
}

/**
 * @brief Tries to initialize node from data stored in EEPROM
 * if data exist and valid
 * @param None
 * @retval HAL status
 */
HAL_StatusTypeDef tryEeprom() {
	nodeSettings_t *eepromSettings = (nodeSettings_t*) FLASH_EEPROM_BASE;
	if (eepromSettings->realFrequency > MIN_FREQUENCY
			&& eepromSettings->realFrequency < MAX_FREQUENCY
			&& eepromSettings->bw < 10 && eepromSettings->cr < 5
			&& eepromSettings->cr > 0 && eepromSettings->sf > 6
			&& eepromSettings->sf < 13 && eepromSettings->power > 9
			&& eepromSettings->power < 21 && eepromSettings->sw != 0x34
			&& eepromSettings->voltageTreshold >= 1.7f
			&& eepromSettings->voltageTreshold <= 3.0f
			&& eepromSettings->workInterval >= MIN_WORK_INTERVAL
			&& eepromSettings->workInterval <= MAX_WORK_INTERVAL
			&& eepromSettings->preamble > 1
			&& eepromSettings->voltageTreshold >= 1.8f
			&& eepromSettings->voltageTreshold <= 3.1f) {
		memcpy((uint8_t*) &settings, (uint8_t*) eepromSettings,
				sizeof(settings));
		initiateSettings();
		return HAL_OK;
	}
	return HAL_ERROR;
}

/**
 * @brief Saves settings from RAM to EEPROM
 * Saves settings by copying var settings to EEPROM
 * @param None
 * @retval None
 */
void saveSettings(nodeSettings_t *settingsPtr) {
	uint8_t i;
	uint32_t *ptr = (uint32_t*) settingsPtr;
	HAL_FLASHEx_DATAEEPROM_Unlock();
	for (i = 0; i < (sizeof(*settingsPtr) + 3) / 4; i++)
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,
		FLASH_EEPROM_BASE + i * 4, *ptr++);
	HAL_FLASHEx_DATAEEPROM_Lock();
}

void setWakeup(uint16_t delay) {
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, recomendedDelay,
	RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
	debugLogInt("Setting WakeUp timer to %d seconds", delay);
}
/**
 * @brief Turns Node into Sleep Mode
 * Set RTC  Wake up timer
 * Switches radio module into sleep mode
 * Switches MCU to Stop mode
 * @param None
 * @retval None
 */
void sleep() {
	HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin | ORANGE_Pin, LED_OFF);
	SX127X_sleep(&myRadio);
	HAL_DBGMCU_DisableDBGStopMode();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

}

/**
 * @Sends current status of Node to Base station
 * @param None
 * @retval None
 */
void sendStatus(nodeStatus_t *status, nodeSettings_t *settingsPtr) {

	txMes->adr = settings.nodeNum;
	txMes->uplink = 1;
	txMes->disarm = status->disarmed;
	txMes->message = MSG_UP_ACKNOWLEDGE;
	txMes->opened = status->opened || status->unconfirmedOpening;
	txMes->powered = status->powered;
	txMes->codedTemperature = getTemperature() * 2.0F + 80;
	txMes->codedVoltage = ((int) (getVoltage() * 10)) - 19;
	status->openedToConfirm = status->opened || status->unconfirmedOpening;
	status->poweredToConfirm = status->powered;
	SX127X_transmitAsync(&myRadio, sizeof(uplinkMessage_t));

}

/**
 * @brief Turns of user pins to decrease power consumption
 * @param None
 * @retval None
 */
void deinitGpio() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = USER1_Pin | USER2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER1_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Turns Off Door Alarm pin and interrupt
 * @param None
 * @retval None
 */
void deinitAlarmInput() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Turns Off External Power detection pin and interrupt
 * @param None
 * @retval None
 */
void deinitPowerInput() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = extPower_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(extPower_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Turns On Door Alarm pin and interrupt
 * @param None
 * @retval None
 */
void initAlarmInput() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Turns On External Power detection pin and interrupt
 * @param None
 * @retval None
 */
void initPowerInput() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = extPower_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(extPower_GPIO_Port, &GPIO_InitStruct);
}
/**
 * @brief Configures Radio module, tries to load custom settings from EEPROM
 * if there is no valid data in EEPROM than loads default settings
 * @param None
 * @retval None
 */
void RadioInit() {
	SX127X_dio_t nss;
	SX127X_dio_t reset;

	SX127X_defaultConfig(&myRadio);
	defaultSettings();
	tryEeprom();
	nss.pin = NSS_Pin;
	nss.port = NSS_GPIO_Port;
	reset.pin = RESET_Pin;
	reset.port = RESET_GPIO_Port;
	SX127X_PortConfig(&myRadio, reset, nss, &hspi1);
	SX127X_init(&myRadio);
	SX127X_config(&myRadio);
}

/**
 * @brief Infinite cycle with receiving test
 * Red Led means module hears LoRa signal
 * Green Led means message received (with valid CRC)
 * @param None
 * @retval None
 */
void ReceivingTest() {
	uint32_t recTime = 0x80000000;
	uint32_t recTime1 = 0x80000000;
	uint32_t recTime2 = 0x80000000;
	bool repeaterMode = false;
	debugLogTime("Receiving test activated");
	while (1) {

		SX127X_Routine(&myRadio);
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

		SX127X_Routine(&myRadio);

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

void ledRoutine(SX127X_t *module) {
	static uint32_t lastBlink = 0;
	if (module->signalDetected && settings.useLed)
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_ON);
	else
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, LED_OFF);

	if (module->status == TX && settings.useLed)
		HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_ON);
	else
		HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, LED_OFF);

	if (HAL_GetTick() - lastBlink > 5000) {
		lastBlink = HAL_GetTick();
	}
	if (HAL_GetTick() - lastBlink < 50 && settings.useLed) {
		HAL_GPIO_WritePin(BLUE_GPIO_Port, ORANGE_Pin | BLUE_Pin, LED_ON);
	}

}

void printInfo(nodeStatus_t *status, uint32_t tm, uint32_t miss) {
#ifdef DEBUG
	uint32_t time = RTC->TR;
	uint32_t date = RTC->DR;
	uint32_t tick = HAL_GetTick();
	printf("Status requested\n");
	printf("SysTick: %10lu\n", tick);
	printf("System time: %02x:%02x:%02x\n", (uint16_t) (time >> 16) & 0xFF,
			(uint16_t) (time >> 8) & 0xFF, (uint16_t) time & 0xFF);
	printf("System date: %x.%02x.%x\n", (uint16_t) date & 0xFF,
			(uint16_t) (date >> 8) & 0x1F,
			((uint16_t) (date >> 16) & 0xFF) + 0x2000);
	printf("Voltage: %d.%02d V\n", (int) getVoltage(),
			((int) (getVoltage() * 100) % 100));
	printf("Temp: %d.%01d C\n", (int) getTemperature(),
			((int) (getTemperature() * 10) % 10));
	printf("Powered: %s\n", status->powered ? "Yes" : "No");
	printf("Powered to confirm: %s\n", status->poweredToConfirm ? "Yes" : "No");
	printf("Disarmed: %s\n", status->disarmed ? "Yes" : "No");
	printf("Door opened: %s\n", status->opened ? "Yes" : "No");
	printf("Opened to confirm: %s\n", status->openedToConfirm ? "Yes" : "No");
	printf("Unconfirmed opening: %s\n",
			status->unconfirmedOpening ? "Yes" : "No");
	printf("Radio status: %s\n", statuses[myRadio.status]);
	printf("Message count: %lu\n", tm);
	printf("Without acknowledge: %lu\n", miss);
	printf("Displayed in : %lu ms\n", HAL_GetTick() - tick);
#endif
}

void showErrorCode(int errNum) {
	for (int i = 0; i < errNum; i++) {
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin | BLUE_Pin, 1);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin | BLUE_Pin, 0);
		HAL_Delay(1000);
	}
}
void startBlink() {
	for (int i = 0; i < 3; i++) {
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin | BLUE_Pin, 1);
		HAL_Delay(200);
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin | BLUE_Pin, 0);
		HAL_Delay(200);
	}
	HAL_Delay(500);
}

void showNumber() {
	for (int i = 0; i < settings.nodeNum / 10; i++) {
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, 1);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(ORANGE_GPIO_Port, ORANGE_Pin, 0);
		HAL_Delay(1000);
	}
	for (int i = 0; i < settings.nodeNum % 10; i++) {
		HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, 1);
		HAL_Delay(800);
		HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, 0);
		HAL_Delay(800);
	}
}

void HAL_Delay(uint32_t Delay) {
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = Delay;

	/* Add a freq to guarantee minimum wait */
	if (wait < HAL_MAX_DELAY) {
		wait += (uint32_t) (uwTickFreq);
	}

	while ((HAL_GetTick() - tickstart) < wait) {
		HAL_WWDG_Refresh(&hwwdg);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_LPTIM1_Init();
  MX_WWDG_Init();
  /* USER CODE BEGIN 2 */

	initUart(&huart1, &myRadio);
	HAL_LPTIM_Counter_Start_IT(&hlptim1, 256 * WATCHDOG_INTERVAL);
	printf("<ANv%lx>\n", SOFTWARE_REVISION);
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk
			| DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk;

	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
	RadioInit();
	recomendedDelay = settings.workInterval;

	flag.rtcAlarm = 1;
	status.poweredConfirmed = HAL_GPIO_ReadPin(extPower_GPIO_Port,
	extPower_Pin);
	status.openedConfirmed = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);
	HAL_DBGMCU_DisableDBGStopMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// PIN MAP: 0-1
	//Receiving test
	if (HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin) == USER2_ACTIVE
			&& HAL_GPIO_ReadPin(USER1_GPIO_Port, USER1_Pin) != USER1_ACTIVE) {
		debugLog("Receiving test activated");
		ReceivingTest();
	}
	//PIN MAP 1-0
	//Ping Test
	if (HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin) != USER1_ACTIVE
			&& HAL_GPIO_ReadPin(USER1_GPIO_Port, USER1_Pin) == USER1_ACTIVE) {
		debugLog("Ping test activated");
		PingTest();

	}

	//PIN MAP 1-1 - Don't sleep
	if (HAL_GPIO_ReadPin(USER2_GPIO_Port, USER2_Pin) == USER2_ACTIVE
			&& HAL_GPIO_ReadPin(USER1_GPIO_Port, USER1_Pin) == USER1_ACTIVE) {
		debugLog("Non sleep mode enabled");
		dontSleep = true;
	}

	debugLog("Power on, test starting");

	if (myRadio.revision == 0) {	//No Radio connection
		debugLog("LoRa module:ERROR!");
		showErrorCode(2);
		while (1)
			;
	}
	debugLogInt("LoRa module: OK, rev: %d", myRadio.revision);

	if ((RCC->CSR & RCC_CSR_LSERDY_Msk) == 0) {	//Crystal failure
		debugLog("Crystal: fail!");
		showErrorCode(3);
		while (1)
			;
	}
	debugLog("Crystal: OK");
	startBlink();
	showNumber();

	deinitGpio();
	flag.rtcAlarm = 1;
	debugLogInt("Node number: %d ", settings.nodeNum);
	debugLogInt("Frequency: %lu", settings.realFrequency);
	debugLogInt("Spreading factor: %d", settings.sf);
	debugLogInt("BandWidth: %d", settings.bw);

	while (1) {
		HAL_WWDG_Refresh(&hwwdg);
		temp = getTemperature();
		if (status.powered
				!= HAL_GPIO_ReadPin(extPower_GPIO_Port, extPower_Pin)) {
			poweredChangeMoment = HAL_GetTick();
			status.powered = HAL_GPIO_ReadPin(extPower_GPIO_Port, extPower_Pin);
		}

		if (flag.statusRequested) {

			flag.statusRequested = 0;

			printInfo(&status, totalMessages, acknowledgeMiss);
		}

		if (flag.rtcAlarm) {
			flag.rtcAlarm = 0;
			if (status.disarmed == 0 || status.powered == 0)
				triesToSend = MAX_RETRIES;

		}


		if (flag.saveSettings) {
			flag.saveSettings = 0;

			debugLogTime("Saving settings to EEPROM");
			initiateSettings(&settings);
			saveSettings(&settings);
		}

		if ((status.openedConfirmed != status.opened
				|| (status.poweredConfirmed != status.powered
						&& HAL_GetTick() - poweredChangeMoment
								> settings.nodeNum * 1200) || triesToSend > 0
				|| status.unconfirmedOpening) && !wfa && !status.disarmed
				&& HAL_GetTick() - myRadio.lastSignalTick
						> 10 + settings.nodeNum * 20 && myRadio.status != TX) {
			if (status.opened)
				status.unconfirmedOpening = true;
			if (triesToSend)
				triesToSend--;

			debugLogTime("Sending status");
			debugLogInt("%d tries left", triesToSend);

			totalMessages++;
			lastTransTime = HAL_GetTick();
			sendStatus(&status, &settings);
			wfa = true;
			setWakeup(recomendedDelay);
		}

		//Got no acknowledge
		if (wfa && HAL_GetTick() - lastTransTime > interval) {
			acknowledgeMiss++;
			uint32_t maxInterval = settings.workInterval * 1000 / 3;
			debugLogTime("Got no acknowledge!");
			wfa = false;
			interval += INTERVAL_STEP + settings.nodeNum * 300;
			interval = (interval > maxInterval) ? maxInterval : interval;
		}

		checkUart();

		if (flag.readConfig) {
			flag.readConfig = 0;

			sendConfig(&settings);
		}

		if (myRadio.readBytes > 0) {
			debugLogTime("Got message...");
			if (myRadio.badCrc == 1) {
				debugLog("Bad CRC!");
				myRadio.readBytes = 0;
			} else if (rxMes->uplink == 0 && rxMes->adr == settings.nodeNum) {
				if (status.disarmed != rxMes->disarm)
					debugLogString("Disarmed changed to %s",
							rxMes->disarm ? "Yes" : "No");

				status.disarmed = rxMes->disarm;
				if (rxMes->codedDelayMSB || rxMes->codedDelayLSB) {

					recomendedDelay = rxMes->codedDelayLSB
							+ (rxMes->codedDelayMSB << 8);
					if (recomendedDelay > 2 * settings.workInterval) {
						debugLogInt(
								"###Recommended delay is too high(%u), returning to 2x work Interval",
								recomendedDelay);
						recomendedDelay = 2 * settings.workInterval;
					} else
						debugLogInt("Recommended delay set to %u",
								recomendedDelay);
				} else {
					debugLogInt("Recommended returned to %u",
							(uint16_t) settings.workInterval);
					recomendedDelay = settings.workInterval;
				}
				setWakeup(recomendedDelay);
				if (rxMes->message == MSG_DOWN_REQUEST) //Request current status
				{
					debugLog("Status requested...sending");
					totalMessages++;
					lastTransTime = HAL_GetTick();
					sendStatus(&status, &settings);
					wfa = true;
					setWakeup(recomendedDelay);
				}

				else if (rxMes->message == MSG_DOWN_ACKNOWLEDGE) {
					debugLog("Acknowledge received");
					if (status.openedToConfirm == 1)
						status.unconfirmedOpening = 0;
					status.openedConfirmed = status.openedToConfirm;
					status.poweredConfirmed = status.poweredToConfirm;
					wfa = false;
					interval = DEFAULT_INTERVAL;
					triesToSend = 0;

				}
			} else {

				if (rxMes->uplink == 0) {
					debugLogInt("It's for %d", rxMes->adr);
					debugLogInt("Delay for it: %d",
							rxMes->codedDelayLSB + (rxMes->codedDelayMSB << 8));
				} else {
					debugLogInt("It's from %d", rxMes->adr);
				}
			}
			myRadio.readBytes = 0;
		}

		SX127X_Routine(&myRadio);

		ledRoutine(&myRadio);

		status.opened = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin)
				|| status.unconfirmedOpening;
		status.powered = HAL_GPIO_ReadPin(extPower_GPIO_Port, extPower_Pin);
		if (!status.powered && !wfa && !myRadio.TXrequest
				&& status.powered == status.poweredConfirmed
				&& status.opened == status.openedConfirmed && !dontSleep
				&& triesToSend < 1) {
			debugLogTime("Sleep...");
			sleep();
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCCEx_EnableLSECSS();
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TempPower_Pin|RESET_Pin|NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLUE_Pin|ORANGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin extPower_Pin */
  GPIO_InitStruct.Pin = D1_Pin|extPower_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TempPower_Pin */
  GPIO_InitStruct.Pin = TempPower_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TempPower_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin NSS_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB14 PB15 PB3
                           PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BLUE_Pin ORANGE_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin|ORANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USER1_Pin USER2_Pin */
  GPIO_InitStruct.Pin = USER1_Pin|USER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	debugLog("Error!");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

