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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart1;

/* Definitions for TaskADC */
osThreadId_t TaskADCHandle;
const osThreadAttr_t TaskADC_attributes = {
  .name = "TaskADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskCAN */
/*osThreadId_t TaskCANHandle;
const osThreadAttr_t TaskCAN_attributes = {
  .name = "TaskCAN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};*/
/* Definitions for TaskSPI */
osThreadId_t TaskSPIHandle;
const osThreadAttr_t TaskSPI_attributes = {
  .name = "TaskSPI",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for messageBuffer_10 */
/*osMessageQueueId_t messageBuffer_10Handle;
const osMessageQueueAttr_t messageBuffer_10_attributes = {
  .name = "messageBuffer_10"
};*/
/* USER CODE BEGIN PV */
MCP2515_HandleTypeDef mcp2515_1;
//canMessage canMessagesBuffer[BUFFER_TX_LENGHT];
//volatile uint8_t mcp2515Int;
//uint8_t canMsgToSend = 0;
MCP2515_canMessage canMessageTx[BUFFER_TX_SPI];
static uint8_t result_mcp2515Init = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartTaskADC(void *argument);
void StartTaskCAN(void *argument);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of messageBuffer_10 */
  //messageBuffer_10Handle = osMessageQueueNew (10, sizeof(uint16_t), &messageBuffer_10_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskADC */
  TaskADCHandle = osThreadNew(StartTaskADC, NULL, &TaskADC_attributes);

  /* creation of TaskCAN */
  //TaskCANHandle = osThreadNew(StartTaskCAN, NULL, &TaskCAN_attributes);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCP2515_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
void canMsgTx(uint8_t* dataToSend, uint32_t* msgID, uint8_t extendedFormat, uint8_t dlc){
	static uint8_t indexTx = 0;
	uint8_t msgId_bf[4];

	if (extendedFormat){
		  msgId_bf[0] = (*msgID >> 3) & 0xFF;  // Estrai il byte alto (MSB)
		  msgId_bf[1] = (*msgID << 5) & 0xE0;         // Estrai il byte basso (LSB)
		  msgId_bf[1] = msgId_bf[1] | 8;
		  msgId_bf[1] = msgId_bf[1] | ((*msgID >> 28) & 0x02);
		  msgId_bf[2] = (*msgID >> 19) & 0xFF;
		  msgId_bf[3] = (*msgID >> 11) & 0xFF;

	  } else{

		msgId_bf[0] = (*msgID >> 3) & 0xFF;  // Estrai il byte alto (MSB)
		msgId_bf[1] = (*msgID << 5) & 0xE0;         // Estrai il byte basso (LSB)
	  }


	  memcpy(canMessageTx[indexTx].msgID, msgId_bf, sizeof(msgId_bf));
	  memcpy(canMessageTx[indexTx].msgData, dataToSend, dlc);
	  canMessageTx[indexTx].dlc = dlc;
	  canMessageTx[indexTx].newMsg = true;

	  indexTx++;
	  if (indexTx >= BUFFER_TX_SPI) {
		  indexTx = 0;
	  }

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
	const TickType_t xFrequency = 5;


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
	uint32_t msgId14 = 0x285;



	//uint32_t start_time, timeADC;
	// Inizializza start_time
	// start_time = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);


	  //start_time = HAL_GetTick();  // Usa SysTick per ottenere il tempo attuale

	  if (result_mcp2515Init == MCP2515_OK){

		  // Incrementa il contatore
		  dataToSend1[0]++;
		  dataToSend2[0]--;
		  dataToSend3[0]+=2;

		  canMsgTx(dataToSend1, &msgId1, false, 8);
		  canMsgTx(dataToSend2, &msgId2, false, 8);
		  canMsgTx(dataToSend3, &msgId3, false, 8);
		  canMsgTx(dataToSend4, &msgId4, false, 8);
		  canMsgTx(dataToSend5, &msgId5, false, 8);
		  canMsgTx(dataToSend6, &msgId6, false, 8);
		  canMsgTx(dataToSend7, &msgId7, false, 8);
		  canMsgTx(dataToSend8, &msgId8, false, 8);
		  canMsgTx(dataToSend9, &msgId9, false, 8);
		  /*canMsgTx(dataToSend10, &msgId10, false, 8);
		  canMsgTx(dataToSend11, &msgId11, false, 8);
		  canMsgTx(dataToSend12, &msgId12, false, 8);
		  canMsgTx(dataToSend13, &msgId13, false, 8);
		  canMsgTx(dataToSend14, &msgId14, false, 8);*/
		 /* char message [10];
			  // Converte l'intero in una stringa
			  sprintf(message, "ADC: %d\r\n", canMessagesBuffer[index].msgData[0]);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
	*/

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

/* USER CODE BEGIN Header_StartTaskCAN */
/**
* @brief Function implementing the TaskCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCAN */
/*void StartTaskCAN(void *argument)
{
  /* USER CODE BEGIN StartTaskCAN */
	/*uint8_t msgId_bf[4];
	static uint8_t indexBfIn = 0;
	static uint8_t indexTxMsg = 0;
	//uint32_t start_time, timeADC;
		// Inizializza start_time
		// start_time = HAL_GetTick();
  /* Infinite loop */
  /*for(;;)
  {
	  if (canMessagesBuffer[indexBfIn].newMsg
			  &&  !canMessageTx[indexTxMsg].sending
			  )  {	// prevedere timeout
		  //start_time = HAL_GetTick();  // Usa SysTick per ottenere il tempo attuale
		  if (canMessagesBuffer[indexBfIn].extended){
			  msgId_bf[0] = (canMessagesBuffer[indexBfIn].msgID >> 3) & 0xFF;  // Estrai il byte alto (MSB)
			  msgId_bf[1] = (canMessagesBuffer[indexBfIn].msgID << 5) & 0xE0;         // Estrai il byte basso (LSB)
			  msgId_bf[1] = msgId_bf[1] | 8;
			  msgId_bf[1] = msgId_bf[1] | ((canMessagesBuffer[indexBfIn].msgID >> 28) & 0x02);
			  msgId_bf[2] = (canMessagesBuffer[indexBfIn].msgID >> 19) & 0xFF;
			  msgId_bf[3] = (canMessagesBuffer[indexBfIn].msgID >> 11) & 0xFF;

		  } else{

			msgId_bf[0] = (canMessagesBuffer[indexBfIn].msgID >> 3) & 0xFF;  // Estrai il byte alto (MSB)
			msgId_bf[1] = (canMessagesBuffer[indexBfIn].msgID << 5) & 0xE0;         // Estrai il byte basso (LSB)
		  }


		  memcpy(canMessageTx[indexTxMsg].msgID, msgId_bf, sizeof(msgId_bf));
		  memcpy(canMessageTx[indexTxMsg].msgData, canMessagesBuffer[indexBfIn].msgData, sizeof(canMessagesBuffer[indexBfIn].msgData));
		  canMessageTx[indexTxMsg].dlc = canMessagesBuffer[indexBfIn].dlc;
		  canMessageTx[indexTxMsg].newMsg = true;

		  canMessagesBuffer[indexBfIn].newMsg = false;
		  /*char message [10];
		  	 // Converte l'intero in una stringa
		  sprintf(message, "index: %d\r\n", indexTxMsg);
		  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
*/
	/*	  indexBfIn++;
		  if (indexBfIn >= BUFFER_TX_LENGHT) {
			  indexBfIn = 0;
		  }
		  indexTxMsg++;
		  if (indexTxMsg >= BUFFER_TX_SPI) {
			indexTxMsg = 0;
		  }

		/* timeADC = HAL_GetTick() - start_time;

		  	  char message [20];
		  	  	  	  // Converte l'intero in una stringa
		  	  	  	  sprintf(message, "time CAN: %lu ms\r\n", timeADC);
		  	  	  	  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);*/

	  //}





  //}
  /* USER CODE END StartTaskCAN */
//}

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

	  uint8_t result = 10;
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
		uint32_t msgId14 = 0x285;

	  	uint32_t start_time, timeADC;
	  	uint32_t start_time2, time2;
	  			// Inizializza start_time
	  			 start_time = HAL_GetTick();
	  			start_time2 = HAL_GetTick();*/
	  			char message [50];

	GPIO_PinState  mcp2515IntState;
  /* Infinite loop */
  for(;;)
  {

	  //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  if (result_mcp2515Init == MCP2515_OK){
		/* timeADC = HAL_GetTick() - start_time;
		 time2 = HAL_GetTick() - start_time2;



		 if (time2>=30000){
			 start_time2  = HAL_GetTick();

			 uint8_t readCTRL;

			 sprintf(message, "Reg: %d-%d-%d\r\n", mcp2515_1.emptyTXBuffer[0], mcp2515_1.emptyTXBuffer[1], mcp2515_1.emptyTXBuffer[2]);
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 sprintf(message, "buffer sts: %d\r\n", canMsg_buffer.status );
			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 /*MCP2515_ReadRegister(&mcp2515_1,0x30, &readCTRL);
			 sprintf(message, "ctrl1: 0x%02X\r\n", readCTRL );
			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x40, &readCTRL);
			 sprintf(message, "ctrl2: 0x%02X\r\n", readCTRL);
			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
			 MCP2515_ReadRegister(&mcp2515_1,0x50, &readCTRL);
			 sprintf(message, "crtl3: 0x%02X\r\n", readCTRL);
			 			 HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

		 }

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




		  mcp2515IntState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);

		  resultHandler = MCP2515_InterruptHandler(&mcp2515_1, mcp2515IntState, &canMsg_buffer);

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



		  	resultSend = MCP2515_SendMessage(&mcp2515_1, &canMsg_buffer, canMessageTx);
		  	if (resultSend != 0){

			 // Converte l'intero in una stringa
			  sprintf(message, "ResultSend: %d\r\n", resultSend);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
		  	}


	     	/*
			 // Converte l'intero in una stringa
			  sprintf(message, "Status: %d\r\n", canMsg_buffer.status);
			  HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);


			  				// Converte l'intero in una stringa
			  			  sprintf(message2, "Complete: %d\r\n", mcp2515_1.transmissionComplete);
			  			  HAL_UART_Transmit(&huart1, (uint8_t*)message2, strlen(message2), HAL_MAX_DELAY);*/



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
