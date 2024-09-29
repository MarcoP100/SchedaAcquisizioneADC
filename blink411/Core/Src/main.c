/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcp2515.h"
#include <string.h>
#include "semphr.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* Definitions for TaskADC */
osThreadId_t TaskADCHandle;
const osThreadAttr_t TaskADC_attributes = {
  .name = "TaskADC",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskSPI */
osThreadId_t TaskSPIHandle;
const osThreadAttr_t TaskSPI_attributes = {
  .name = "TaskSPI",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
MCP2515_HandleTypeDef mcp2515_1;
//canMessage canMessagesBuffer[BUFFER_TX_LENGHT];
//volatile uint8_t mcp2515Int;
//uint8_t canMsgToSend = 0;
MCP2515_canMessage canMessageTx[BUFFER_TX_SPI];
static uint8_t result_mcp2515Init = 1;
SemaphoreHandle_t xMutex;
volatile bool adcInUse = false;
volatile bool spiInUse = false;
static uint32_t msg14 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartTaskADC(void *argument);
void StartTaskSPI(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//mcp2515Int = false;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  //xSemaphore = xSemaphoreCreateBinary();
  //xSemaphoreGive(xSemaphore);
  xMutex = xSemaphoreCreateMutex();
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskADC */
  TaskADCHandle = osThreadNew(StartTaskADC, NULL, &TaskADC_attributes);

  /* creation of TaskSPI */
  TaskSPIHandle = osThreadNew(StartTaskSPI, NULL, &TaskSPI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Accende il LED */
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // A seconda del LED potrebbe essere GPIO_PIN_SET
    //HAL_Delay(500); // Aspetta 500ms

    /* Spegne il LED */
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  // A seconda del LED potrebbe essere GPIO_PIN_RESET
    //HAL_Delay(500); // Aspetta 500ms

     /* Invia il messaggio tramite USART1 */
    //HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    /*uint8_t data = 0xAA;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);  // CS basso
	  HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // CS alto
	  HAL_Delay(2000);*/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
/*
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
//__HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedOnBoard_GPIO_Port, LedOnBoard_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LedOnBoard_Pin */
  GPIO_InitStruct.Pin = LedOnBoard_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedOnBoard_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP2515_INT_Pin */
  GPIO_InitStruct.Pin = MCP2515_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCP2515_INT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Verifica che questo callback sia per il tuo hspi
    if (hspi == &hspi1) {
        //MCP2515_SetTransmissionComplete(&mcp2515_1, 1);
    	mcp2515_1.transmissionComplete = true;
        //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        //vTaskNotifyGiveFromISR(xTaskHandleSPI, &xHigherPriorityTaskWoken);
        //portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi1) {
    	//MCP2515_SetTransmissionComplete(&mcp2515_1, 1);
    	mcp2515_1.transmissionComplete = true;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi1) {
    	//MCP2515_SetTransmissionComplete(&mcp2515_1, 1);
    	mcp2515_1.transmissionComplete = true;
    }
}

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_6)
    {
        // Codice per gestire l'interrupt del MCP2515
        mcp2515Int++;
        //printf("Interrupt\n");
       /* char message [20];
       sprintf(message, "Interrupt\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);*/
    //}
//}


int __io_putchar(int ch) {
    // Invia il carattere tramite USART (ad esempio huart1)
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
  }

  int _write(int file, char *ptr, int len) {
      for (int i = 0; i < len; i++) {
          __io_putchar(*ptr++);
      }
      return len;
  }
void printByteArray(uint8_t *array, int size) {
    printf("Array content: ");
    for (int i = 0; i < size; i++) {
        printf("0x%02X ", array[i]);  // Stampa ciascun byte in formato esadecimale
    }
    printf("\n");  // Aggiunge una nuova linea alla fine

    /* uso
    int arraySize = sizeof(writeMessage) / sizeof(writeMessage[0]);
    printByteArray(writeMessage, arraySize);*/
}
void canMsgTx(canMessage* messageToSend, MCP2515_canMessage* messageBuffer){

	uint8_t msgId_bf[4];

	if (messageToSend->extended){
		  msgId_bf[0] = (messageToSend->msgID >> 3) & 0xFF;  // Estrai il byte alto (MSB)
		  msgId_bf[1] = (messageToSend->msgID << 5) & 0xE0;         // Estrai il byte basso (LSB)
		  msgId_bf[1] = msgId_bf[1] | 8;
		  msgId_bf[1] = msgId_bf[1] | ((messageToSend->msgID >> 28) & 0x02);
		  msgId_bf[2] = (messageToSend->msgID >> 19) & 0xFF;
		  msgId_bf[3] = (messageToSend->msgID >> 11) & 0xFF;

	  } else{

		msgId_bf[0] = (messageToSend->msgID >> 3) & 0xFF;  // Estrai il byte alto (MSB)
		msgId_bf[1] = (messageToSend->msgID << 5) & 0xE0;         // Estrai il byte basso (LSB)
		msgId_bf[2] = 0x00;
		msgId_bf[3] = 0x00;
	  }


	  memcpy(messageBuffer->msgID, msgId_bf, sizeof(msgId_bf));
	  memcpy(messageBuffer->msgData, messageToSend->msgData, messageToSend->dlc);
	  messageBuffer->dlc = messageToSend->dlc;
	  messageBuffer->newMsg = true;

	  if ((messageBuffer->msgID[0] != 2) ||
			 ( messageBuffer->msgID[1] != 0 &&
			  messageBuffer->msgID[1] != 96 &&
			  messageBuffer->msgID[1] != 64 &&
			  messageBuffer->msgID[1] != 32)){
		  msg14 += 1;
	  }


}


void Read_ADC_Polling(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t* value) {



        HAL_ADC_ConfigChannel(hadc, sConfig);

        // Avvia la conversione
        HAL_ADC_Start(hadc);

        // Aspetta che la conversione sia completata
        if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
            // Leggi il valore convertito
        	*value = HAL_ADC_GetValue(hadc);

        }


        // Ferma la conversione (opzionale, ma per sicurezza)
        HAL_ADC_Stop(hadc);

}

float scaleValue(float input, float min_input, float max_input, float min_output, float max_output) {
    // Mappatura lineare del valore di input all'intervallo di output
    float output = ((input - min_input) / (max_input - min_input)) * (max_output - min_output) + min_output;
    return output;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskADC */
/**
  * @brief  Function implementing the TaskADC thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskADC */
void StartTaskADC(void *argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = 5;//pdMS_TO_TICKS(5);;


	float min_input = 0.0;     // Minimo valore di tensione (0V)
	float max_input = 4096;     // Massimo valore di tensione (5V)
	float min_output = 0.0;    // Minimo valore di dbar (0 dbar)
	float max_output = 100.0;  // Massimo valore di dbar (100 dbar)


	uint32_t adcValues[8] = {0};
	uint32_t ui_pressureValue_dbar[8] = {0};
	ADC_ChannelConfTypeDef sConfig = {0};
	canMessage canMessage[4] = {0};
	static uint8_t indexTx = 0;
	//uint32_t start_time, timeADC;
	// Inizializza start_time
	// start_time = HAL_GetTick();

	uint16_t value_to_send = 0;

  /* Infinite loop */
  for(;;)
  {

	  //if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {

	  //start_time = HAL_GetTick();  // Usa SysTick per ottenere il tempo attuale

	  if (result_mcp2515Init == MCP2515_OK&&
			spiInUse == false){
			 //(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)) {
		  adcInUse = true;
		  // Esegui la lettura dei canali ADC
		  for (int i = 0; i < 8; i++) {
		          // Seleziona il canale da leggere
		          sConfig.Channel = ADC_CHANNEL_0 + i;
		          sConfig.Rank = 1;
		          sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		          Read_ADC_Polling(&hadc1, &sConfig, &adcValues[i]);

		  }

		  for (int i = 0; i < 8; i++) {
			  float input = (float)adcValues[i];
			  float outputDbar = scaleValue(input, min_input, max_input, min_output, max_output);

			  if (outputDbar < 0) {
				  ui_pressureValue_dbar[i] = 0;  // Saturazione a 0 se il valore è negativo
			  } else if (outputDbar > 65535) {
				  ui_pressureValue_dbar[i] = 65535;  // Saturazione al massimo valore se eccede 65535
			  } else {
				  ui_pressureValue_dbar[i] = (uint32_t)roundf(outputDbar);  // Conversione diretta se è nei limiti
			  }

		  }

		  /********/



		  //0x010
		  canMessage[0].msgID = 0x010;
		  canMessage[0].extended = false;
		  canMessage[0].dlc = 8;

		  for (int m = 0; m < 8; m++){
			  canMessage[0].msgData[m] = 0x0;
		  }

		  //adcValues[0] = 0x00110011;
		  value_to_send = (uint16_t)adcValues[0];
		  canMessage[0].msgData[0] = value_to_send;
		  canMessage[0].msgData[1] = value_to_send >> 8;

		  //adcValues[1] = 0x22332233;
		  value_to_send = (uint16_t)adcValues[1];
		  canMessage[0].msgData[2] = value_to_send;
		  canMessage[0].msgData[3] = value_to_send >> 8;

		  //adcValues[2] = 0x44554455;
		  value_to_send = (uint16_t)adcValues[2];
		  canMessage[0].msgData[4] = value_to_send;
		  canMessage[0].msgData[5] = value_to_send >> 8;

		  //adcValues[3] = 0x66776677;
		  value_to_send = (uint16_t)adcValues[3];
		  canMessage[0].msgData[6] = value_to_send;
		  canMessage[0].msgData[7] = value_to_send >> 8;
		  /*canMessage[0].msgData[0] = 10;
		  		  canMessage[0].msgData[1] = 20;
		  		  canMessage[0].msgData[2] = 30;
		  		  canMessage[0].msgData[3] = 40;
		  		  canMessage[0].msgData[4] = 50;
		  		  canMessage[0].msgData[5] = 60;
		  		  canMessage[0].msgData[6] = 70;
		  		  canMessage[0].msgData[7] = 80;*/

		  canMsgTx(&canMessage[0], &canMessageTx[indexTx]);

		  indexTx++;
		  if (indexTx >= BUFFER_TX_SPI) {
			  indexTx = 0;
		  }

  		 /********/

		  //0x011
		  canMessage[1].msgID = 0x011;
		  canMessage[1].extended = false;
		  canMessage[1].dlc = 8;

		  for (int m = 0; m < 8; m++){
			  canMessage[1].msgData[m] = 0x0;
		  }

		  //adcValues[4] = 0x88998899;
		  value_to_send = (uint16_t)adcValues[4];
		  canMessage[1].msgData[0] = value_to_send;
		  canMessage[1].msgData[1] = value_to_send >> 8;

		  //adcValues[5] = 0xaabbaabb;
		  value_to_send = (uint16_t)adcValues[5];
		  canMessage[1].msgData[2] = value_to_send;
		  canMessage[1].msgData[3] = value_to_send >> 8;

		  //adcValues[6] = 0xccddccdd;
		  value_to_send = (uint16_t)adcValues[6];
		  canMessage[1].msgData[4] = value_to_send;
		  canMessage[1].msgData[5] = value_to_send >> 8;

		  //adcValues[7] = 0xeeffeeff;
		  value_to_send = (uint16_t)adcValues[7];
		  canMessage[1].msgData[6] = value_to_send;
		  canMessage[1].msgData[7] = value_to_send >> 8;
		  /*canMessage[1].msgData[0] = 10;
		  		  		  canMessage[1].msgData[1] = 20;
		  		  		  canMessage[1].msgData[2] = 30;
		  		  		  canMessage[1].msgData[3] = 40;
		  		  		  canMessage[1].msgData[4] = 50;
		  		  		  canMessage[1].msgData[5] = 60;
		  		  		  canMessage[1].msgData[6] = 70;
		  		  		  canMessage[1].msgData[7] = 80;*/

		  canMsgTx(&canMessage[1], &canMessageTx[indexTx]);

		  indexTx++;
		  if (indexTx >= BUFFER_TX_SPI) {
			  indexTx = 0;
		  }

		 /********/
		  //0x012
		  canMessage[2].msgID = 0x012;
		  canMessage[2].extended = false;
		  canMessage[2].dlc = 8;

		  for (int m = 0; m < 8; m++){
			  canMessage[2].msgData[m] = 0x0;
		  }

		  //ui_pressureValue_dbar[0] = 0x00110011;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[0];
		  canMessage[2].msgData[0] = value_to_send;
		  canMessage[2].msgData[1] = value_to_send >> 8;

		  //ui_pressureValue_dbar[1] = 0x22332233;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[1];
		  canMessage[2].msgData[2] = value_to_send;
		  canMessage[2].msgData[3] = value_to_send >> 8;

		  //ui_pressureValue_dbar[2] = 0x44554455;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[2];
		  canMessage[2].msgData[4] = value_to_send;
		  canMessage[2].msgData[5] = value_to_send >> 8;

		  //ui_pressureValue_dbar[3] = 0x66776677;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[3];
		  canMessage[2].msgData[6] = value_to_send;
		  canMessage[2].msgData[7] = value_to_send >> 8;


		  /*canMessage[2].msgData[0] = 10;
		  		  		  canMessage[2].msgData[1] = 20;
		  		  		  canMessage[2].msgData[2] = 30;
		  		  		  canMessage[2].msgData[3] = 40;
		  		  		  canMessage[2].msgData[4] = 50;
		  		  		  canMessage[2].msgData[5] = 60;
		  		  		  canMessage[2].msgData[6] = 70;
		  		  		  canMessage[2].msgData[7] = 80;*/
  		  canMsgTx(&canMessage[2], &canMessageTx[indexTx]);

		  indexTx++;
		  if (indexTx >= BUFFER_TX_SPI) {
			  indexTx = 0;
		  }
		  /***********/
		  //0x013
		  canMessage[3].msgID = 0x013;
		  canMessage[3].extended = false;
		  canMessage[3].dlc = 8;

		  for (int m = 0; m < 8; m++){
			  canMessage[3].msgData[m] = 0x0;
		  }

		  //ui_pressureValue_dbar[4] = 0x88998899;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[4];
		  canMessage[3].msgData[0] = value_to_send;
		  canMessage[3].msgData[1] = value_to_send >> 8;

		  //ui_pressureValue_dbar[5] = 0xaabbaabb;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[5];
		  canMessage[3].msgData[2] = value_to_send;
		  canMessage[3].msgData[3] = value_to_send >> 8;

		  //ui_pressureValue_dbar[6] = 0xccddccdd;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[6];
		  canMessage[3].msgData[4] = value_to_send;
		  canMessage[3].msgData[5] = value_to_send >> 8;

		  //ui_pressureValue_dbar[7] = 0xeeffeeff;
		  value_to_send = (uint16_t)ui_pressureValue_dbar[7];
		  canMessage[3].msgData[6] = value_to_send;
		  canMessage[3].msgData[7] = value_to_send >> 8;
		  /*canMessage[3].msgData[0] = 10;
						  canMessage[3].msgData[1] = 20;
						  canMessage[3].msgData[2] = 30;
						  canMessage[3].msgData[3] = 40;
						  canMessage[3].msgData[4] = 50;
						  canMessage[3].msgData[5] = 60;
						  canMessage[3].msgData[6] = 70;
						  canMessage[3].msgData[7] = 80;*/
			  canMsgTx(&canMessage[3], &canMessageTx[indexTx]);

		  indexTx++;
		  if (indexTx >= BUFFER_TX_SPI) {
			  indexTx = 0;
		  }





		 /* char message [20];
		  // Converte l'intero in una stringa
		  sprintf(message, "index: %d\r\n", indexTx);
		  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);*/
		  adcInUse = false;
		  //xSemaphoreGive(xMutex);

		  // Rilascia il semaforo quando l'operazione è completata
		  //xSemaphoreGive(xSemaphore);

		 /* char message [10];
			  // Converte l'intero in una stringa
			  sprintf(message, "ADC: %d\r\n", canMessagesBuffer[index].msgData[0]);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
	*/
	  //}
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
	  }
	 // HAL_GPIO_TogglePin(LedOnBoard_GPIO_Port, LedOnBoard_Pin);

	  /*timeADC = HAL_GetTick() - start_time;
	  char message [20];
	  // Converte l'intero in una stringa
	  sprintf(message, "time adc: %d ms\r\n", timeADC);
	  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);*/
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskSPI */
/**
* @brief Function implementing the TaskSPI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSPI */
void StartTaskSPI(void *argument)
{
  /* USER CODE BEGIN StartTaskSPI */

	MCP2515_MessageBuffer canMsg_buffer;


	// Inizializzazione del dispositivo MCP2515
	  //HAL_Delay(500);
	  initBuffer(&canMsg_buffer);


	  uint8_t dummyData = 0x00;
	  HAL_SPI_Transmit(&hspi1, &dummyData, 1, HAL_MAX_DELAY); // Trasmissione dummy
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // CS alto


	  const char* resMcp2515_msg;
	  result_mcp2515Init = MCP2515_Init(&mcp2515_1, GPIOB, GPIO_PIN_13, &hspi1, CAN0_BAUDRATE, true);
	  switch ( result_mcp2515Init){

	    case MCP2515_OK:
	      resMcp2515_msg = "Inizializzazione mcp2515 ok\n";
	      break;

	    case MCP2515_FAIL:
	      resMcp2515_msg = "Inizializzazione mcp2515 fallita\n";
	      break;

	    case MCP2515_RESET_FAIL:
	      resMcp2515_msg = "Reset mcp2515 fallito\n";
	      break;

	    case MCP2515_SET_BAUDRATE_FAIL:
	      resMcp2515_msg = "Set baudrate mcp2515 fallito\n";
	      break;

	    case MCP2515_SET_MODE_FAIL:
	      resMcp2515_msg = "Set modalita operativa mcp2515 fallita\n";
	      break;

	    case MCP2515_RESET_TIMEOUT:
	      resMcp2515_msg = "MCP2515_RESET_TIMEOUT\n";
	      break;

	    case MCP2515_READ_TIMEOUT_1:
	      resMcp2515_msg = "MCP2515_READ_TIMEOUT_1\n";
	      break;

	    case MCP2515_READ_TIMEOUT_2:
	      resMcp2515_msg = "MCP2515_READ_TIMEOUT_2\n";
	      break;

	    case MCP2515_BAUDRATE_READ_FAIL:
	      resMcp2515_msg = "MCP2515_BAUDRATE_READ_FAIL\n";
	      break;

	    case MCP2515_BAUDRATE_NOT_OK:
	      resMcp2515_msg = "MCP2515_BAUDRATE_NOT_OK\n";
	      break;

	    default:
	      resMcp2515_msg = "Errore non riconosciuto mcp2515\n";
	      break;
	  }

	  HAL_UART_Transmit(&huart1, (uint8_t*)resMcp2515_msg, strlen(resMcp2515_msg), HAL_MAX_DELAY);

	  uint8_t resultHandler = 0;
	  uint8_t resultSend = 0;

	 // uint8_t result = 10;
	 /* static uint8_t index = 0;
	  static uint8_t dataToSend1[8] = {0x56, 0x26, 0x88, 0x12, 0x67, 0x99, 0xAA, 0x88};
	static uint8_t dataToSend2[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	static uint8_t dataToSend3[8] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
	static uint8_t dataToSend4[8] = {0x3F, 0xA2, 0xB7, 0x19, 0xE4, 0x56, 0x8C, 0x22};
	static uint8_t dataToSend5[8] = {0x7E, 0xD4, 0x11, 0x89, 0xB5, 0x23, 0x6A, 0xFC};
	static uint8_t dataToSend6[8] = {0x4B, 0xC3, 0x9F, 0x02, 0x71, 0xD8, 0xA6, 0x5D};
	static uint8_t dataToSend7[8] = {0x99, 0x28, 0x73, 0xC4, 0xF6, 0x13, 0xB1, 0x2E};
	static uint8_t dataToSend8[8] = {0xEF, 0x5A, 0x64, 0x92, 0x3B, 0x87, 0xD3, 0x41};
	static uint8_t dataToSend9[8] = {0xAA, 0x38, 0x5C, 0x72, 0x49, 0xF1, 0xBE, 0x6D};
	static uint8_t dataToSend10[8] = {0x7E, 0xD4, 0x11, 0x89, 0xB5, 0x23, 0x6A, 0xFC};
		static uint8_t dataToSend11[8] = {0x4B, 0xC3, 0x9F, 0x02, 0x71, 0xD8, 0xA6, 0x5D};
		static uint8_t dataToSend12[8] = {0x99, 0x28, 0x73, 0xC4, 0xF6, 0x13, 0xB1, 0x2E};
		static uint8_t dataToSend13[8] = {0xEF, 0x5A, 0x64, 0x92, 0x3B, 0x87, 0xD3, 0x41};
		static uint8_t dataToSend14[8] = {0xAA, 0x38, 0x5C, 0x72, 0x49, 0xF1, 0xBE, 0x6D};
	uint32_t msgId1 = 0x200;
	uint32_t msgId2 = 0x210;
	uint32_t msgId3 = 0x220;
	uint32_t msgId4 = 0x230;
	uint32_t msgId5 = 0x240;
	uint32_t msgId6 = 0x250;
	uint32_t msgId7 = 0x260;
	uint32_t msgId8 = 0x270;
	uint32_t msgId9 = 0x280;
	uint32_t msgId10 = 0x281;
		uint32_t msgId11 = 0x282;
		uint32_t msgId12 = 0x283;
		uint32_t msgId13 = 0x284;
		uint32_t msgId14 = 0x285;*/

	  	//uint32_t start_time, timeADC;
	  	uint32_t start_time2, time2;
	  			// Inizializza start_time
	  			 //start_time = HAL_GetTick();
	  			start_time2 = HAL_GetTick();
	  			char message [50];

	GPIO_PinState  mcp2515IntState;
	//static BaseType_t taskADCIsSuspended = pdFALSE;
  /* Infinite loop */
  for(;;)
  {

	  //if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
	  //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  if (result_mcp2515Init == MCP2515_OK){
		// timeADC = HAL_GetTick() - start_time;
		/* time2 = HAL_GetTick() - start_time2;



		 if (time2>=12000 && canMsg_buffer.status == TRANSMISSION_IDLE){
			 start_time2  = HAL_GetTick();

			 uint8_t readCTRL;

			 sprintf(message, "--Reg: %d-%d-%d\r\n", mcp2515_1.emptyTXBuffer[0], mcp2515_1.emptyTXBuffer[1], mcp2515_1.emptyTXBuffer[2]);
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 sprintf(message, "--buffer sts: %d\r\n", canMsg_buffer.status );
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x30, &readCTRL);
			 sprintf(message, "--ctrl1: 0x%02X\r\n", readCTRL );
			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x40, &readCTRL);
			 sprintf(message, "--ctrl2: 0x%02X\r\n", readCTRL);
			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x50, &readCTRL);
			 sprintf(message, "--crtl3: 0x%02X\r\n", readCTRL);
			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 sprintf(message, "--state: 0x%02X\r\n", mcp2515_1.hspi->State);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			  MCP2515_ReadRegister(&mcp2515_1,MCP2515_CANINTF_MSG, &readCTRL);
			  sprintf(message, "--flag: 0x%02X\r\n", readCTRL );
			  			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			MCP2515_ReadRegister(&mcp2515_1,MCP2515_CANINTE_MSG, &readCTRL);
			  sprintf(message, "--flag: 0x%02X\r\n", readCTRL );
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x2D, &readCTRL);
			  sprintf(message, "--eflg: 0x%02X\r\n", readCTRL );
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x1C, &readCTRL);
			  sprintf(message, "--TEC: 0x%02X\r\n", readCTRL );
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x1D, &readCTRL);
			  sprintf(message, "--REC: 0x%02X\r\n", readCTRL );
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 sprintf(message, "-------------------\r\n");
			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

		 }*/
/*
		  if (timeADC >=5){

			  start_time = HAL_GetTick();
			  /*sprintf(message, "Flag: %d\r\n", mcp2515Int);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

			  //sprintf(message, "Reg: %d-%d-%d,   buffer sts: %d\r\n", mcp2515_1.emptyTXBuffer[0], mcp2515_1.emptyTXBuffer[1], mcp2515_1.emptyTXBuffer[2],canMsg_buffer.status );
			  //HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

			  dataToSend1[0] ++;
			  canMsgTx(dataToSend1, &msgId1, false, 8);
			  canMsgTx(dataToSend2, &msgId2, false, 8);
			  canMsgTx(dataToSend3, &msgId3, false, 8);
			  canMsgTx(dataToSend4, &msgId4, false, 8);
			  canMsgTx(dataToSend5, &msgId5, false, 8);
			  canMsgTx(dataToSend6, &msgId6, false, 8);
			  canMsgTx(dataToSend7, &msgId7, false, 8);
			  canMsgTx(dataToSend8, &msgId8, false, 8);
			  canMsgTx(dataToSend9, &msgId9, false, 8);
			 /* canMsgTx(dataToSend10, &msgId10, false, 8);
			  			  canMsgTx(dataToSend11, &msgId11, false, 8);
			  			  canMsgTx(dataToSend12, &msgId12, false, 8);
			  			  canMsgTx(dataToSend13, &msgId13, false, 8);
			  			  canMsgTx(dataToSend14, &msgId14, false, 8);

		}*/


		  if (!spiInUse && !adcInUse){
			  mcp2515IntState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);

			  resultHandler = MCP2515_InterruptHandler(&mcp2515_1, mcp2515IntState, &canMsg_buffer);
		  }



		  	 /* if (resultHandler != 0){

			  	// Converte l'intero in una stringa
			  sprintf(message, "Result: %d\r\n", resultHandler);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			  // Converte l'intero in una stringa
			  	//	  	sprintf(message, "Flag: %d\r\n", mcp2515Int);
			  	//	  	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
		  	  }*/
		  	 /* if (mcp2515Int==true){
		  		sprintf(message, "Flag ON\r\n");
		  		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
		  	  }*/


			  //if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE){
				 // do{
		  	 if (!adcInUse){
						resultSend = MCP2515_SendMessage(&mcp2515_1, &canMsg_buffer, canMessageTx);
						if (canMsg_buffer.status != TRANSMISSION_IDLE) {
							spiInUse = true;
						}
						else
							spiInUse = false;



						/*if (resultSend != 0){

						 // Converte l'intero in una stringa
						  sprintf(message, "ResultSend: %d\r\n", resultSend);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
						}*/
		  	 }
				 // }while(canMsg_buffer.status != TRANSMISSION_IDLE);
				/*if (canMsg_buffer.status != TRANSMISSION_IDLE && taskADCIsSuspended == pdFALSE) {
					vTaskSuspend(TaskADCHandle);
					taskADCIsSuspended = pdTRUE;
				} else if (canMsg_buffer.status == TRANSMISSION_IDLE && taskADCIsSuspended == pdTRUE) {
					vTaskResume(TaskADCHandle);
					taskADCIsSuspended = pdFALSE;
				}*/
				/*
				 // Converte l'intero in una stringa
				  sprintf(message, "Status: %d\r\n", canMsg_buffer.status);
				  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);


								// Converte l'intero in una stringa
							  sprintf(message2, "Complete: %d\r\n", mcp2515_1.transmissionComplete);
							  HAL_UART_Transmit(&huart1, (uint8_t*)message2, strlen(message2), HAL_MAX_DELAY);*/
					//  xSemaphoreGive(xSemaphore);

			 // }

	     /*if (canMsg_buffer.status == TRANSMISSION_END){
	    	char message [20];
	    	// Converte l'intero in una stringa
	    	sprintf(message, "Msg: %d\r\n", canMsg_buffer.buffer);
	    	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
	     }*/

	  }
	  //char *message = "SPI\n";
	  //	  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
  }
  /* USER CODE END StartTaskSPI */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
