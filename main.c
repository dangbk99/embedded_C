/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
float temp_global = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for temp_Queue */
osMessageQueueId_t temp_QueueHandle;
const osMessageQueueAttr_t temp_Queue_attributes = {
  .name = "temp_Queue"
};
/* USER CODE BEGIN PV */
void ADC_init()
{
	__HAL_RCC_ADC1_CLK_ENABLE();
	uint32_t* SMPR1 = (uint32_t*)0x4001200c;
	*SMPR1 |= (0b111<<18);
	uint32_t* JSQR = (uint32_t*)0x40012038;
	*JSQR |=(16<<15);  //set channel 16
	uint32_t* CR1 = (uint32_t*)0x40012004;
	uint32_t* CR2 = (uint32_t*)0x40012008;
	*CR2 |=(0b01<<20)|(1<<0);

	uint32_t* CCR = (uint32_t*)0x40012304;
	*CCR |= (1<<23);
}
uint16_t Read_ADC()
{
	uint32_t* CR2 = (uint32_t*)0x40012008;
	*CR2 |=(1<<22); //start injected
	uint32_t* SR = (uint32_t*)0x40012000;
	while( ((*SR>>2)&1)==0); // wait conversion complete
	*SR &= ~(1<<2); //clear status
	uint32_t* JDR1 = (uint32_t*)0x4001203c;
	return *JDR1;
}

void UART_init()
{
	//PA2 = Tx; PA3 = Rx
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
	*GPIOA_MODER &= ~(0b1111<<4);
	*GPIOA_MODER |= (0b10<<4) |(0b10<<6);
	uint32_t* GPIOA_AFRL = (uint32_t*)0x40020020;
	*GPIOA_AFRL &= ~(0xff<<8);
	*GPIOA_AFRL |= (7<<8) |(7<<12);

	__HAL_RCC_USART2_CLK_ENABLE();
	uint32_t* USART2_BRR = (uint32_t*)0x40004408;
	*USART2_BRR = (104<<4)|3; //set baud rate

	uint32_t* USART2_CR1 = (uint32_t*)0x4000440c;
	*USART2_CR1 |= (1<<13)|(1<<2) |(1<<3)|(1<<5);
	uint32_t* USART2_CR3 = (uint32_t*)0x40004414;
	* USART2_CR3 |= (1<<6);
	//uint32_t* NVIC_ISER1 = (uint32_t*)0xe000e104;
	//*NVIC_ISER1 |=(0b1<<6); //38-32
}
void UART_send_bytes(char data)
{
	uint32_t* USART2_DR = (uint32_t*)0x40004404;
	uint32_t* USART2_SR = (uint32_t*)0x40004400;
	while(((*USART2_SR >> 7) & 1) != 1 );
	*USART2_DR = data;
	while(((*USART2_SR >> 6) & 1) != 1 );
}
void UART_send_multy_bytes(char* arr)
{
	int size = strlen(arr);
	for (int i =0; i<size;i++)
	{
		UART_send_bytes(arr[i]);
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void read_temp_ss_task(void *argument);

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
  /* creation of temp_Queue */
  temp_QueueHandle = osMessageQueueNew (16, sizeof(float), &temp_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(read_temp_ss_task, NULL, &myTask02_attributes);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	UART_init();
	float temp = 0;
  /* Infinite loop */
  for(;;)
  {
	//temp = temp_global;
	osMessageQueueGet(temp_QueueHandle, &temp, osPriorityNormal, 10000 );
	char msg[32]={0};
	char frac =0;
	frac = (int)((temp - (int)temp)*100);
	sprintf(msg,"Nhiet_do: %d.%d*C\r\n", (int)temp, frac);
	UART_send_multy_bytes(msg);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_read_temp_ss_task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_temp_ss_task */
void read_temp_ss_task(void *argument)
{
  /* USER CODE BEGIN read_temp_ss_task */
	ADC_init();
	uint16_t raw_data = 0;
	float vin = 0;
	float temp = 0;
  /* Infinite loop */
  for(;;)
  {
	  raw_data = Read_ADC();
	  vin = (raw_data*3)/4095.0;
	  temp = ((vin - 0.76)/0.0025)+25;
	  osMessageQueuePut(temp_QueueHandle, &temp,osPriorityNormal, 10000 );
	  //temp_global = temp;
	  osDelay(1000);
  }
  /* USER CODE END read_temp_ss_task */
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
