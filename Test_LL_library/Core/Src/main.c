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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define ROWS 8
#define COLS 8



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void led_blink_example_code(void);
void UART2_TX_string(char* str);
static void inline delay_us(unsigned int delay);
void write_serial_in_parallel_out_register_plus(unsigned char data);
void write_serial_in_parallel_out_register_minus(unsigned char data);
unsigned char rowToBinary(unsigned char (*ptr)[8], int row);
void led_serial_in_parallel_out_example_1(void);
void heart_example_1(void);
void heart_example_2(void);
void heart_example_3(void);
void heart_example_4(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int time_gap=1;
char buffer_[50];
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  int count=0;
  char buffer_[15];
  char buffer_2[15];
  unsigned char dotMatrixBoard[ROWS][COLS] = {
		  {1, 1, 1, 1, 1, 1, 1, 1},
		  {1, 1, 1, 1, 1, 1, 1, 1},
		  {1, 1, 1, 1, 1, 1, 1, 1},
		  {1, 1, 1, 1, 1, 1, 1, 1},
		  {1, 1, 1, 1, 1, 1, 1, 1},
		  {1, 1, 1, 1, 1, 1, 1, 1},
		  {1, 1, 1, 1, 1, 1, 1, 1},
		  {1, 1, 1, 1, 1, 1, 1, 1}
  };
  //write_serial_in_parallel_out_register(data_8bit_ex12);
//  write_serial_in_parallel_out_register_minus(0b10011001);
//  write_serial_in_parallel_out_register_plus(0b01000000);
//
//  write_serial_in_parallel_out_register_minus(0b00000000);
//  write_serial_in_parallel_out_register_plus(0b00100000);

  //WRITE_REG(TIM2->CCR1, 100);

  //LL_TIM_DisableIT_UPDATE(TIM2);
  //LL_TIM_EnableIT_CC1(TIM2);  //TIM2->DIER register CC1IE bit. (ENABLE)
  //LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); //CCER register CC1E bit. (ENABLE)



  /* choose to which to Off color */
  write_serial_in_parallel_out_register_minus_red(0b11111111);
  //write_serial_in_parallel_out_register_minus_yellow(0b11111111);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //row:0
	  write_serial_in_parallel_out_register_plus(0b10000000);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 0));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 0));
	  LL_mDelay(1);
	  //row:1
	  write_serial_in_parallel_out_register_plus(0b01000000);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 1));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 1));
	  LL_mDelay(1);
	  //row:2
	  write_serial_in_parallel_out_register_plus(0b00100000);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 2));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 2));
	  LL_mDelay(1);
	  //row:3
	  write_serial_in_parallel_out_register_plus(0b00010000);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 3));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 3));
	  LL_mDelay(1);
	  //row:4
	  write_serial_in_parallel_out_register_plus(0b00001000);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 4));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 4));
	  LL_mDelay(1);
	  //row:5
	  write_serial_in_parallel_out_register_plus(0b00000100);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 5));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 5));
	  LL_mDelay(1);
	  //row:6
	  write_serial_in_parallel_out_register_plus(0b00000010);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 6));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 6));
	  LL_mDelay(1);
	  //row:7
	  write_serial_in_parallel_out_register_plus(0b00000001);
	  write_serial_in_parallel_out_register_minus_yellow(rowToBinary(dotMatrixBoard, 7));
	  //write_serial_in_parallel_out_register_minus_red(rowToBinary(dotMatrixBoard, 7));
	  LL_mDelay(1);


//	  sprintf(buffer_, "%d", count);
//	  UART2_TX_string("count : ");
//	  UART2_TX_string(buffer_);
//	  UART2_TX_string("\r\n");

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }  //while loop
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 63;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, MR_3_Pin|MR_2_Pin|DSA_3_Pin|DSA_1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, CP_2_Pin|DSA_2_Pin|CP_1_Pin|MR_1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CP_3_GPIO_Port, CP_3_Pin);

  /**/
  GPIO_InitStruct.Pin = MR_3_Pin|MR_2_Pin|DSA_3_Pin|DSA_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CP_2_Pin|DSA_2_Pin|CP_1_Pin|MR_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CP_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(CP_3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void led_blink_example_code(void){

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);  //1 second delay.
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);  //1 second delay.

}

void UART2_TX_string(char* str){
	while(*str){
		while(!LL_USART_IsActiveFlag_TXE(USART2));

		LL_USART_TransmitData8(USART2, *str);
		str++;
	}
}

void led_serial_in_parallel_out_example_1(void){
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5); //data=high
	LL_mDelay(100);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);  //clock pulse 1
	LL_mDelay(100);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);  //clock pulse 2
	LL_mDelay(100);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);  //clock pulse 3
	LL_mDelay(100);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);

	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5); //data=low
	LL_mDelay(100);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);  //clock pulse 1
	LL_mDelay(100);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);  //clock pulse 2
	LL_mDelay(100);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);  //clock pulse 3
	LL_mDelay(100);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
	LL_mDelay(100);
}

void write_serial_in_parallel_out_register_plus(unsigned char data){
	/*
	 *                Q0  Q1  Q2  Q3  Q4  Q5  Q6  Q7(------>)
	 *      data      1    1   0   0   0  0   0   1
	 *
	 *
	 */
	//all reset
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);

	//Q7
	unsigned int bitValue = bitRead(data, 0);

	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 1

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

	//Q6
	bitValue = bitRead(data, 1);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 2

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

	//Q5
	bitValue = bitRead(data, 2);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 3

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

	//Q4
	bitValue = bitRead(data, 3);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 4

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

	//Q3
	bitValue = bitRead(data, 4);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 5

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

	//Q2
	bitValue = bitRead(data, 5);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 6

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

	//Q1
	bitValue = bitRead(data, 6);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 7

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

	//Q0
	bitValue = bitRead(data, 7);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);  //clock pulse 8

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);

	delay_us(1);

}

void write_serial_in_parallel_out_register_minus(unsigned char data){
	/*
	 *                Q0  Q1  Q2  Q3  Q4  Q5  Q6  Q7(------>)
	 *      data      1    1   0   0   0  0   0   1
	 *
	 *
	 */
	//all reset
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
	//LL_mDelay(2);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);

	//Q7
	unsigned int bitValue = bitRead(data, 0);

	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 1
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	//Q6
	bitValue = bitRead(data, 1);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 2
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	//Q5
	bitValue = bitRead(data, 2);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(2);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 3
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	//Q4
	bitValue = bitRead(data, 3);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(2);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 4
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	//Q3
	bitValue = bitRead(data, 4);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(2);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 5
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	//Q2
	bitValue = bitRead(data, 5);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(2);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 6
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	//Q1
	bitValue = bitRead(data, 6);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(2);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 7
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	//Q0
	bitValue = bitRead(data, 7);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6); //data=high
			break;

	}
	//LL_mDelay(2);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);  //clock pulse 8
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	//LL_mDelay(1);
	//delayus(100);
	//LL_mDelay(time_gap);
	delay_us(1);

}

void write_serial_in_parallel_out_register_minus_yellow(unsigned char data){
	/*
	 *                Q0  Q1  Q2  Q3  Q4  Q5  Q6  Q7(------>)
	 *      data      1    1   0   0   0  0   0   1
	 *
	 *
	 */
	//all reset
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);

	//Q7
	unsigned int bitValue = bitRead(data, 0);

	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);  //clock pulse 1

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

	//Q6
	bitValue = bitRead(data, 1);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);  //clock pulse 2

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

	//Q5
	bitValue = bitRead(data, 2);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);  //clock pulse 3

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

	//Q4
	bitValue = bitRead(data, 3);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);   //clock pulse 4

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

	//Q3
	bitValue = bitRead(data, 4);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);  //clock pulse 5

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

	//Q2
	bitValue = bitRead(data, 5);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);  //clock pulse 6

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

	//Q1
	bitValue = bitRead(data, 6);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);  //clock pulse 7

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

	//Q0
	bitValue = bitRead(data, 7);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); //data=high
			break;

	}

	//delay_us(1);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);  //clock pulse 8

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

	delay_us(1);

}

void write_serial_in_parallel_out_register_minus_red(unsigned char data){
	/*
	 *                Q0  Q1  Q2  Q3  Q4  Q5  Q6  Q7(------>)
	 *      data      1    1   0   0   0  0   0   1
	 *
	 *
	 */
	//all reset
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);

	//Q7
	unsigned int bitValue = bitRead(data, 0);

	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);  //clock pulse 1

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

	//Q6
	bitValue = bitRead(data, 1);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);  //clock pulse 2

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

	//Q5
	bitValue = bitRead(data, 2);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);  //clock pulse 3

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

	//Q4
	bitValue = bitRead(data, 3);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);   //clock pulse 4

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

	//Q3
	bitValue = bitRead(data, 4);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);  //clock pulse 5

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

	//Q2
	bitValue = bitRead(data, 5);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);  //clock pulse 6

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

	//Q1
	bitValue = bitRead(data, 6);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);  //clock pulse 7

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

	//Q0
	bitValue = bitRead(data, 7);
	switch(bitValue){
		case 0 :
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=low
			break;

		case 1 :
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9); //data=high
			break;

	}

	delay_us(1);

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);  //clock pulse 8

	delay_us(1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

	delay_us(1);

}


unsigned char rowToBinary(unsigned char (*ptr)[8], int row) {
	unsigned char binaryValue = 0;
	for (int col = 0; col < COLS; col++) {
		binaryValue |= (ptr[row][col] << (COLS - 1 - col));
	}
	binaryValue = ~binaryValue;

	return binaryValue;
}


static void inline delay_us(unsigned int delay){
	/*
	 * USAGE : please use delay variable that is ccr register.
	 */
	char buffer_delay_us[20];  //make buffer


	TIM2->CNT = 0; //clear

	LL_TIM_EnableCounter(TIM2); //CR1 register CEN bit (ENABLE)

	while(TIM2->CNT < delay){
		//Loop until the counter value is greater or equal to the delay
//		sprintf(buffer_delay_us, "%ld", TIM2->CNT);
//
//		UART2_TX_string("TIM2->CNT : ");
//		UART2_TX_string(buffer_delay_us);
//		UART2_TX_string("\r\n");

	}

	LL_TIM_DisableCounter(TIM2);   //disable.

}

void heart_example_1(void){

	//row : 0

	 //row : 1
	write_serial_in_parallel_out_register_minus(0b10011001);
	write_serial_in_parallel_out_register_plus(0b01000000);
	LL_mDelay(2);
	//row : 2
	write_serial_in_parallel_out_register_minus(0b01111110);
	write_serial_in_parallel_out_register_plus(0b00100000);
	LL_mDelay(2);
	//row : 3
	write_serial_in_parallel_out_register_minus(0b01111110);
	write_serial_in_parallel_out_register_plus(0b00010000);
	LL_mDelay(2);
//		//row : 4
	write_serial_in_parallel_out_register_minus(0b10111101);
	write_serial_in_parallel_out_register_plus(0b00001000);
	LL_mDelay(2);
//		//row : 5
	write_serial_in_parallel_out_register_minus(0b11011011);
	write_serial_in_parallel_out_register_plus(0b00000100);
	LL_mDelay(2);
	//row : 6
	write_serial_in_parallel_out_register_minus(0b11100111);
	write_serial_in_parallel_out_register_plus(0b00000010);
	LL_mDelay(2);


	//row : 7



}

void heart_example_2(void){
	 //row : 0

	 //row : 1
	write_serial_in_parallel_out_register_minus(0b10011001);
	write_serial_in_parallel_out_register_plus(0b01000000);
	LL_mDelay(2);
	//row : 2
	write_serial_in_parallel_out_register_minus(0b00000000);
	write_serial_in_parallel_out_register_plus(0b00100000);
	LL_mDelay(2);
	//row : 3
	write_serial_in_parallel_out_register_minus(0b00111100);
	write_serial_in_parallel_out_register_plus(0b00010000);
	LL_mDelay(2);
//		//row : 4
	write_serial_in_parallel_out_register_minus(0b10011001);
	write_serial_in_parallel_out_register_plus(0b00001000);
	LL_mDelay(2);
//		//row : 5
	write_serial_in_parallel_out_register_minus(0b11000011);
	write_serial_in_parallel_out_register_plus(0b00000100);
	LL_mDelay(2);
	//row : 6
	write_serial_in_parallel_out_register_minus(0b11100111);
	write_serial_in_parallel_out_register_plus(0b00000010);
	LL_mDelay(2);


	//row : 7
}

void heart_example_3(void){
	 //row : 0

	 //row : 1
	write_serial_in_parallel_out_register_minus(0b10011001);
	write_serial_in_parallel_out_register_plus(0b01000000);
	LL_mDelay(2);
	//row : 2
	write_serial_in_parallel_out_register_minus(0b00000000);
	write_serial_in_parallel_out_register_plus(0b00100000);
	LL_mDelay(2);
	//row : 3
	write_serial_in_parallel_out_register_minus(0b00000000);
	write_serial_in_parallel_out_register_plus(0b00010000);
	LL_mDelay(2);
//		//row : 4
	write_serial_in_parallel_out_register_minus(0b10000001);
	write_serial_in_parallel_out_register_plus(0b00001000);
	LL_mDelay(2);
//		//row : 5
	write_serial_in_parallel_out_register_minus(0b11000011);
	write_serial_in_parallel_out_register_plus(0b00000100);
	LL_mDelay(2);
	//row : 6
	write_serial_in_parallel_out_register_minus(0b11100111);
	write_serial_in_parallel_out_register_plus(0b00000010);
	LL_mDelay(2);


	//row : 7
}

void heart_example_4(void){
	 //row : 0

////	 //row : 1
	write_serial_in_parallel_out_register_plus(0b10000000);
	write_serial_in_parallel_out_register_minus_yellow(0b00000000);
	LL_mDelay(5);
////	//row : 2
	write_serial_in_parallel_out_register_plus(0b00000000);
	write_serial_in_parallel_out_register_minus_yellow(0b00000000);
	LL_mDelay(5);
////	//row : 3
	write_serial_in_parallel_out_register_plus(0b00100000);
	write_serial_in_parallel_out_register_minus_yellow(0b00000000);
	LL_mDelay(5);
////	//row : 4
	write_serial_in_parallel_out_register_plus(0b00000000);
	write_serial_in_parallel_out_register_minus_yellow(0b00000000);
	LL_mDelay(5);
////	//row : 5
	write_serial_in_parallel_out_register_plus(0b00001000);
	write_serial_in_parallel_out_register_minus_yellow(0b00000000);
	LL_mDelay(5);
////	//row : 6
	write_serial_in_parallel_out_register_plus(0b00000000);
	write_serial_in_parallel_out_register_minus_yellow(0b00000000);
	LL_mDelay(5);
////
////
////	//row : 7
	write_serial_in_parallel_out_register_plus(0b00000010);
	write_serial_in_parallel_out_register_minus_yellow(0b00000000);
	LL_mDelay(5);
}



/* USER CODE END 4 */

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
