/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "i2c-lcd.h"
#include "stdio.h"


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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

osThreadId defaultTaskHandle;
osThreadId DHT11Handle;
osThreadId Anh_SangHandle;
osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
osMessageQId myQueue03Handle;
osMessageQId myQueue04Handle;
osMessageQId myQueue05Handle;
osMessageQId myQueue06Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);
void DHT11_FULL(void const * argument);
void Anh_Sang_FULL(void const * argument);

/* USER CODE BEGIN PFP */
//delay function in microseconds
void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim4))<time);
}




/*
//Finction to display temperature
void Display_Temp (int Temp)
{
	char str[20] = {0};
	lcd_put_cur(0,0);
	
	sprintf (str, "Temp:%d", Temp);
	lcd_send_string(str);
	lcd_send_data('C');
}

////function to display RH
void Display_Rh (int Rh)
{
	char str[20] = {0};
	lcd_put_cur(0,10);
	
	sprintf (str, "Rh:%d", Rh);
	lcd_send_string(str);
	lcd_send_data('%');
}
*/
//variables declarations
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

uint32_t Temperature = 0;
uint32_t Humidity = 0;
uint8_t Presence = 0;


//function to set the pins as outputs
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//function to set pin as input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //can be changed to PULLUP if no data is received from the pin
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/***************************DHT11 FUNCTIONS BEGIN HERE**********************************************/
//Define the pin and the port for DHT11 Sensor
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

//Function to send the start signal
void DHT11_Start (void) 
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN); //set the dht pin as output
	/***********************************************/
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1); //initialize with data pin high
	HAL_Delay(1000); //wait for 1000 milliseconds
	/***********************************************/

	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0); //pull the pin low
	delay(18000); //wait 18 milliseconds
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1); //pull the pin high
	delay(20); //wait for 20 microseconds
	Set_Pin_Input(DHT11_PORT, DHT11_PIN); //set the pin as input
}

//dh11 function to check response
//uint8_t DHT11_Check_Response (void) 
//{
//	uint8_t Response = 0;
//	delay(40); 
//	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) 
//	{
//		delay(80); 
//		if((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1; 
//		else Response = -1; 
//	}
//	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); 
//	
//	return Response;
//}

//function to read data from dht11 signal pin
uint8_t DHT11_Read (void)
{
	uint8_t i, j;
	for (j=0;j<8;j++)
	{
		while(!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to change to high
		delay(40); //wait for 40 microseconds
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) //if the pin is low
		{
			i&= ~(1<<(7-j)); //write 0
		}
		else i|= (1<<(7-j)); //if the pin is high write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to go low
	}
	
	return i;
}
//////////////////////////////////////DHT0///////////////////////
/* USER CODE END 0 
void Display_Temp5 (int Temp5)
{
	char str[20] = {0};
	lcd_put_cur(1,0);
	
	sprintf (str, "Temp:%d", Temp5);
	lcd_send_string(str);
	lcd_send_data('C');
}

//function to display RH
void Display_Rh5 (int Rh5)
{
	char str[20] = {0};
	lcd_put_cur(1,10);
	
	sprintf (str, "Rh:%d", Rh5);
	lcd_send_string(str);
	lcd_send_data('%');
}
*/
//variables declarations
uint8_t Rh5_byte1, Rh5_byte2, Temp5_byte1, Temp5_byte2;
uint16_t SUM5, RH5, TEMP5;

uint32_t Temperature5 = 0;
uint32_t Humidity5 = 0;
uint8_t Presence5 = 0;


//function to set the pins as outputs
void Set_Pin_Output5(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//function to set pin as input
void Set_Pin_Input5(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //can be changed to PULLUP if no data is received from the pin
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/***************************DHT11 FUNCTIONS BEGIN HERE**********************************************/
//Define the pin and the port for DHT11 Sensor
#define DHT11_PORT GPIOA
#define DHT11_PIN5 GPIO_PIN_0 

//Function to send the start signal
void DHT11_Start5 (void) 
{
	Set_Pin_Output5 (DHT11_PORT, DHT11_PIN5); //set the dht pin as output
	/***********************************************/
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN5, 1); //initialize with data pin high
	HAL_Delay(1000); //wait for 1000 milliseconds
	/***********************************************/

	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN5, 0); //pull the pin low
	delay(18000); //wait 18 milliseconds
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN5, 1); //pull the pin high
	delay(20); //wait for 20 microseconds
	Set_Pin_Input5(DHT11_PORT, DHT11_PIN5); //set the pin as input
}

//dh11 function to check response
//uint8_t DHT11_Check_Response5 (void) 
//{
//	uint8_t Response5 = 0;
//	delay(40); 
//	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN5))) 
//	{
//		delay(80); 
//		if((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN5))) Response5 = 1; 
//		else Response5 = -1; 
//	}
//	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN5))); 
//	
//	return Response5;
//}

//function to read data from dht11 signal pin
uint8_t DHT11_Read5 (void)
{
	uint8_t i, j;
	for (j=0;j<8;j++)
	{
		while(!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN5))); //wait for the pin to change to high
		delay(40); //wait for 40 microseconds
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN5))) //if the pin is low
		{
			i&= ~(1<<(7-j)); //write 0
		}
		else i|= (1<<(7-j)); //if the pin is high write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN5))); //wait for the pin to go low
	}
	
	return i;
}
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
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start(&htim4);
	
	lcd_init(); //initialize lcd
	lcd_send_string("INITIALIZING.."); //display string on lcd
	HAL_Delay(2000); //wait for 2 seconds
	lcd_clear(); //clear lcd display
	
  /* USER CODE END 2 */

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
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* definition and creation of myQueue02 */
  osMessageQDef(myQueue02, 16, uint16_t);
  myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);

  /* definition and creation of myQueue03 */
  osMessageQDef(myQueue03, 16, uint16_t);
  myQueue03Handle = osMessageCreate(osMessageQ(myQueue03), NULL);

  /* definition and creation of myQueue04 */
  osMessageQDef(myQueue04, 16, uint16_t);
  myQueue04Handle = osMessageCreate(osMessageQ(myQueue04), NULL);

  /* definition and creation of myQueue05 */
  osMessageQDef(myQueue05, 16, uint16_t);
  myQueue05Handle = osMessageCreate(osMessageQ(myQueue05), NULL);

  /* definition and creation of myQueue06 */
  osMessageQDef(myQueue06, 16, uint16_t);
  myQueue06Handle = osMessageCreate(osMessageQ(myQueue06), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DHT11 */
  osThreadDef(DHT11, DHT11_FULL, osPriorityNormal, 0, 128);
  DHT11Handle = osThreadCreate(osThread(DHT11), NULL);

  /* definition and creation of Anh_Sang */
  osThreadDef(Anh_Sang, Anh_Sang_FULL, osPriorityNormal, 0, 128);
  Anh_SangHandle = osThreadCreate(osThread(Anh_Sang), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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


  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
		lcd_init();
  /* Infinite loop */
  for(;;)
  {
		
	xQueueReceive(myQueue01Handle, &Temperature, portMAX_DELAY);
	xQueueReceive(myQueue02Handle, &Humidity, portMAX_DELAY);
	xQueueReceive(myQueue03Handle, &Temperature5, portMAX_DELAY);
	xQueueReceive(myQueue04Handle, &Humidity5, portMAX_DELAY);
		
	char str[20] = {0};
	
	lcd_put_cur(0,0);
	sprintf (str, "Temp:%d", Temperature);
	lcd_send_string(str);
	lcd_send_data('C');
	
	lcd_put_cur(0,10);
	sprintf (str, "Rh:%d", Humidity);
	lcd_send_string(str);
	lcd_send_data('%');
	
	lcd_put_cur(1,0);
	sprintf (str, "Temp:%d", Temperature5);
	lcd_send_string(str);
	lcd_send_data('C');
	
	lcd_put_cur(1,10);
	sprintf (str, "Rh:%d", Humidity5);
	lcd_send_string(str);
	lcd_send_data('%');
	
  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DHT11_FULL */
/**
* @brief Function implementing the DHT11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DHT11_FULL */
void DHT11_FULL(void const * argument)
{
  /* USER CODE BEGIN DHT11_FULL */
	
  /* Infinite loop */
  for(;;)
  {
		//Display_Temp(Temperature);
		//Display_Rh(Humidity);
		
		
		DHT11_Start();
		//Presence = DHT11_Check_Response(); 
		
		
		Rh_byte1 = DHT11_Read ();
		Rh_byte2 = DHT11_Read ();
		Temp_byte1 = DHT11_Read ();
		Temp_byte2 = DHT11_Read ();
		SUM = DHT11_Read ();
		
		TEMP = Temp_byte1;
		RH = Rh_byte1;
		
		Temperature = (float) TEMP;
		Humidity = (float) RH;
		
    //// DHT1////
		//Display_Temp5(Temperature5);
		//Display_Rh5(Humidity5);
		
		/*******DHT11*********/
		DHT11_Start5();
		//Presence5 = DHT11_Check_Response5(); //record the response from the sensor
		
		//Five bytes of data
		Rh5_byte1 = DHT11_Read5 ();
		Rh5_byte2 = DHT11_Read5 ();
		Temp5_byte1 = DHT11_Read5 ();
		Temp5_byte2 = DHT11_Read5 ();
		SUM5 = DHT11_Read5 ();
		
		TEMP5 = Temp5_byte1;
		RH5 = Rh5_byte1;
		
		Temperature5 = (float) TEMP5;
		Humidity5 = (float) RH5;
		
		xQueueSend(myQueue01Handle, &Temperature, portMAX_DELAY);
		xQueueSend(myQueue02Handle, &Humidity, portMAX_DELAY);
		xQueueSend(myQueue03Handle, &Temperature5, portMAX_DELAY);
		xQueueSend(myQueue04Handle, &Humidity5, portMAX_DELAY);
		
    osDelay(1);
  }
  /* USER CODE END DHT11_FULL */
}

/* USER CODE BEGIN Header_Anh_Sang_FULL */
/**
* @brief Function implementing the Anh_Sang thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Anh_Sang_FULL */
void Anh_Sang_FULL(void const * argument)
{
  /* USER CODE BEGIN Anh_Sang_FULL */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Anh_Sang_FULL */
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
