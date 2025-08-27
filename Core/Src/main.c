/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "NRF24.h"

#include "util/util.h"

#include "stdio.h"
#include "string.h"
#include "transmitter.h"
#include "ota.h"
#include "logger.h"
#include "adc.h"

#include "tasks/blinkled.h"
#include "tasks/txdummy.h"
#include "tasks/receive_poll.h"
#include "tasks/ping.h"
#include "tasks/stick_read.h"
#include "tasks/tx_command.h"

#include "error/error.h"

#include "stddef.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KERNEL_HAL		1
#define KERNEL_RTOS		2

#define BUTTON_DELAY_BOUND	300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t KERNEL_MODE = KERNEL_HAL;
#define CHANNEL 0x45
uint8_t address[5] = { 0, 0, 0, 0, 1 };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SendDummy */
osThreadId_t SendDummyHandle;
const osThreadAttr_t SendDummy_attributes = {
  .name = "SendDummy",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal6,
};
/* Definitions for nrf24_receive */
osThreadId_t nrf24_receiveHandle;
const osThreadAttr_t nrf24_receive_attributes = {
  .name = "nrf24_receive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for pingSend */
osThreadId_t pingSendHandle;
const osThreadAttr_t pingSend_attributes = {
  .name = "pingSend",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for StickRead */
osThreadId_t StickReadHandle;
const osThreadAttr_t StickRead_attributes = {
  .name = "StickRead",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for TemperatureLoad */
osThreadId_t TemperatureLoadHandle;
const osThreadAttr_t TemperatureLoad_attributes = {
  .name = "TemperatureLoad",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TransmitCommand */
osThreadId_t TransmitCommandHandle;
const osThreadAttr_t TransmitCommand_attributes = {
  .name = "TransmitCommand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for ErrorPollTask */
osThreadId_t ErrorPollTaskHandle;
const osThreadAttr_t ErrorPollTask_attributes = {
  .name = "ErrorPollTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for change_permit */
osMessageQueueId_t change_permitHandle;
const osMessageQueueAttr_t change_permit_attributes = {
  .name = "change_permit"
};
/* Definitions for motor_x */
osMessageQueueId_t motor_xHandle;
const osMessageQueueAttr_t motor_x_attributes = {
  .name = "motor_x"
};
/* Definitions for MCUTemp */
osMessageQueueId_t MCUTempHandle;
const osMessageQueueAttr_t MCUTemp_attributes = {
  .name = "MCUTemp"
};
/* Definitions for dataMutex */
osMutexId_t dataMutexHandle;
const osMutexAttr_t dataMutex_attributes = {
  .name = "dataMutex"
};
/* USER CODE BEGIN PV */
extern uint32_t _ota_begin, _ota_end, _ota_loadaddr;
Dummy_t rxdata;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void send_dummy(void *argument);
void receive_poll(void *argument);
void ping_send(void *argument);
void stick_read(void *argument);
void fetch_temp(void *argument);
void start_transmit_cmd(void *argument);
void start_error_poll(void *argument);

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
	set_log_level(  LEVEL_INFO | LEVEL_ERROR  );
//	ota_init();
	copy_text();
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

  HAL_GPIO_WritePin(  USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET  );

  csn_high();
  HAL_Delay( 5 );
  ce_low();
  nrf24_init();

  nrf24_auto_ack_all( auto_ack );
  nrf24_en_ack_pld( enable );
  nrf24_en_dyn_ack( enable );
  nrf24_dpl( disable );

  nrf24_set_crc( no_crc, _1byte );

  nrf24_tx_pwr( n18dbm );
  nrf24_data_rate( _250kbps );
  nrf24_set_channel( CHANNEL );
  nrf24_set_addr_width( 5 );

  nrf24_set_rx_dpl( 0, disable );
  nrf24_set_rx_dpl( 1, disable );
  nrf24_set_rx_dpl( 2, disable );
  nrf24_set_rx_dpl( 3, disable );
  nrf24_set_rx_dpl( 4, disable );
  nrf24_set_rx_dpl( 5, disable );

  nrf24_pipe_pld_size( 0, 32 );

  nrf24_open_tx_pipe( address );
  nrf24_open_rx_pipe( 0, address );
  nrf24_stop_listen();

  ce_high();

  HAL_Delay( 5 );
  transmit_text(  "BLA BLA BLA BLA BLA BLA BLA BLA BLA BLA BLA BLA BLA BLA BLA"  );
  HAL_Delay( 1000 );

  Command_t cmd;
  memset( &cmd, 0, sizeof( cmd ) );
  cmd.type = PACKET_COMMAND;
  cmd.M1 = 2;
  cmd.M2 = 137;
  cmd.M3 = 3;
  cmd.M4 = 45;
  transmit( ( Dummy_t* ) &cmd );
  HAL_Delay( 1000 );

  Ping_t p;
  memset( &p, 0, 32 );
  p.type = PACKET_PING;
  transmit( ( Dummy_t* ) &p );

  KERNEL_MODE = KERNEL_RTOS;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of dataMutex */
  dataMutexHandle = osMutexNew(&dataMutex_attributes);

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
  /* creation of change_permit */
  change_permitHandle = osMessageQueueNew (16, sizeof(uint32_t), &change_permit_attributes);

  /* creation of motor_x */
  motor_xHandle = osMessageQueueNew (1, sizeof(uint32_t), &motor_x_attributes);

  /* creation of MCUTemp */
  MCUTempHandle = osMessageQueueNew (1, sizeof(uint16_t), &MCUTemp_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SendDummy */
  SendDummyHandle = osThreadNew(send_dummy, NULL, &SendDummy_attributes);

  /* creation of nrf24_receive */
  nrf24_receiveHandle = osThreadNew(receive_poll, NULL, &nrf24_receive_attributes);

  /* creation of pingSend */
  pingSendHandle = osThreadNew(ping_send, NULL, &pingSend_attributes);

  /* creation of StickRead */
  StickReadHandle = osThreadNew(stick_read, NULL, &StickRead_attributes);

  /* creation of TemperatureLoad */
  TemperatureLoadHandle = osThreadNew(fetch_temp, NULL, &TemperatureLoad_attributes);

  /* creation of TransmitCommand */
  TransmitCommandHandle = osThreadNew(start_transmit_cmd, NULL, &TransmitCommand_attributes);

  /* creation of ErrorPollTask */
  ErrorPollTaskHandle = osThreadNew(start_error_poll, NULL, &ErrorPollTask_attributes);

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
  while (  1  )
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_8;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_SELECT_Pin|CE_USER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_SELECT_Pin CE_USER_Pin */
  GPIO_InitStruct.Pin = CS_SELECT_Pin|CE_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, 100);
	return len;
}


u32 btn1_tick = 0;

void HAL_GPIO_EXTI_Callback(  uint16_t GPIO_Pin  ) {

	if(  GPIO_Pin  ==  BTN1_Pin  &&  HAL_GPIO_ReadPin(  BTN1_GPIO_Port, BTN1_Pin  )  ==  GPIO_PIN_RESET  &&  (  osKernelGetTickCount() - btn1_tick  >  BUTTON_DELAY_BOUND  )  ) {

		printf(  "Interrupt triggered\r\n"  );

		HAL_GPIO_TogglePin(  LED1_GPIO_Port, LED1_Pin  );

		btn1_tick  =  osKernelGetTickCount();

	}

}


void ota_init(void) {
	uint32_t* src =	&_ota_loadaddr;
	uint32_t* dst = &_ota_begin;
	while(dst < &_ota_end) {
		*dst++ = *src++;
	}
}

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
	while(  1  ) {

		osDelay(  1  );

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_send_dummy */
/**
* @brief Function implementing the SendDummy thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send_dummy */
void send_dummy(void *argument)
{
  /* USER CODE BEGIN send_dummy */
	vTaskSuspend(  NULL  );
	transmit_dummy();
  /* USER CODE END send_dummy */
}

/* USER CODE BEGIN Header_receive_poll */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receive_poll */
void receive_poll(void *argument)
{
  /* USER CODE BEGIN receive_poll */
	vTaskSuspend(  NULL  );
	rx_poll(  (  Dummy_t*  )  &rxdata  );
  /* USER CODE END receive_poll */
}

/* USER CODE BEGIN Header_ping_send */
/**
* @brief Function implementing the pingSend thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ping_send */
void ping_send(void *argument)
{
  /* USER CODE BEGIN ping_send */
//	vTaskSuspend(  NULL  );
  transmit_ping();
  /* USER CODE END ping_send */
}

/* USER CODE BEGIN Header_stick_read */
/**
* @brief Function implementing the StickRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stick_read */
void stick_read(void *argument)
{
  /* USER CODE BEGIN stick_read */
  stick_adc_read();
  /* USER CODE END stick_read */
}

/* USER CODE BEGIN Header_fetch_temp */
/**
* @brief Function implementing the TemperatureLoad thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fetch_temp */
void fetch_temp(void *argument)
{
  /* USER CODE BEGIN fetch_temp */
  /* Infinite loop */
  for(  ;;  )
  {

	  u32 val = read_adc_channel(  ADC_CHANNEL_TEMPSENSOR, ADC_SAMPLETIME_480CYCLES  );

	  float temp = (  (  float  ) val /  (  float  )  4095  ) * 3.3f;
	  temp = (  (  temp - 0.76f  ) / 0.0025f  ) + 25.0f;

	  printf(  "Temp: %d\r\n",  (  int  )  temp  );


	  osDelay(  1000  );

  }
  /* USER CODE END fetch_temp */
}

/* USER CODE BEGIN Header_start_transmit_cmd */
/**
* @brief Function implementing the TransmitCommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_transmit_cmd */
void start_transmit_cmd(void *argument)
{
  /* USER CODE BEGIN start_transmit_cmd */
  txcommand_poll();
  /* USER CODE END start_transmit_cmd */
}

/* USER CODE BEGIN Header_start_error_poll */
/**
* @brief Function implementing the ErrorPollTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_error_poll */
void start_error_poll(void *argument)
{
  /* USER CODE BEGIN start_error_poll */
	error_poll();
  /* USER CODE END start_error_poll */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
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
#ifdef USE_FULL_ASSERT
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
