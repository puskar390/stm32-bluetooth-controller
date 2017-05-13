/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : control motors and LED via bluetooh using mobile app
  * Programmer		   : Puskar Pandey
  * Date			   : 26 April 2016
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdlib.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
HAL_StatusTypeDef rc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

GPIO_InitTypeDef GPIO_InitStruct;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);

void TIM1_BRK_TIM15_IRQHandler(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ledInit(void);
void Init(void);
void PutChar(unsigned char c);
void GotoXY(unsigned char x, unsigned char y);
void PutStr(char *str);
void ClrScr(void);
void display_shift(void);
void display_cursor(int status);
void dcRun(int dir);
void dcInit(void);
void dcStop(void);
void encoderInit(void);
void encoder(void);
void toggleLed(void);
void writeLed(int led, int state);

/* Private variables -----------------------------------------------*/
uint32_t counter15 = 0;
int ledFlag = 0;		//checks use to star LED pattern

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  ledInit();
  Init();
  dcInit();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();

  /* USER CODE BEGIN 2 */


    uint8_t string[10] = "START\r\n";
    uint8_t received[20] = {0};
    uint8_t encoderFlag = 0;   //checks whether to display encoder value
    uint8_t *bufferPtr;
    int direction = 1;
    bufferPtr = received;

    char state = '0';
    //char preState = '2';
    HAL_UART_Transmit(&huart1, string, 10, 100);
  //char str[10];

    ClrScr();
    display_cursor(0);
    GotoXY(0,0);
    PutStr("WELLCOME");


    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

  /* USER CODE END 2 */


   HAL_TIM_Base_Start_IT(&htim15);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  /* USER CODE BEGIN 3 */
	  HAL_UART_Receive(&huart1, received, 10, 200);
	  state = (char)received[0];
		  //HAL_UART_Transmit(&huart1, string, 10, 100);

	  switch(state){

		  case '0':
			  break;

		  case '1':
			  ledFlag = 1;
			  GotoXY(0,0);
			  PutStr("LED ON       ");
			  break;

		  case '2':
			  ledFlag = 0;
			  for(int i=0; i<8; i++){
				  writeLed(i, 0);
			  }
			  GotoXY(0,0);
			  PutStr("LED OFF      ");
			  break;

		  case '3':
			  dcStop();
			  direction =1;
			  dcRun(direction);
			  GotoXY(0,0);
			  PutStr("Motor ON FWD ");
			  break;

		  case '4':
			  dcStop();
			  GotoXY(0,0);
			  PutStr("Motor OFF     ");
			  break;

		  case '5':
			  encoderFlag = 1;
			  break;

		  case '6':
			  dcStop();
			  direction = 2;
			  dcRun(direction);
			  GotoXY(0,0);
			  PutStr("Motor ON RWD ");
			  break;

		  case '7':
			  GotoXY(0,0);
			  PutStr("MIN Speed    ");
			  TIM1->CCR1 = 300;
			  break;

		  case '8':
			  GotoXY(0,0);
			  PutStr("MAX Speed    ");
			  TIM1->CCR1 = 1000;
			  break;

		  default:
			  state = '0';
			  break;

	  }
	  //turn on and of the encoder
	  if (encoderFlag == 1){
		  encoder( );
	  }

	 *bufferPtr = 0;

  }

  /* USER CODE END 3 */
}




/*Name: ledInit
 *Description: Initializing GPIOE pins for led
 *Parameter: None
 *Return: None
 * */

/*Initializing GPIOE pins for led*/
void ledInit(){
	__GPIOE_CLK_ENABLE();
	GPIO_InitStruct.Pin = ( GPIO_PIN_8 |GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 );
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}


/*Name: dcRun
 *Description: Starts the motor to run
 *Parameter: direction for the motor
 *Return: None
 * */
void dcRun( int dir ){
	//Forward direction
	if(dir == 1){
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET); //enable the motor
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_SET); //disable the motor
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	}
	//reverse direction
	if(dir == 2){
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET); //enable the motor
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_RESET); //disable the motor
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	}
	return;
}

/* Name: writeLed
 * Description: Changes the state of the LED
 * Parameters: LED number and its state (0 or 1)
 *Return: None
 */
void writeLed(int led, int state ){
	const uint32_t LEDs[] = { GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

	HAL_GPIO_WritePin(GPIOE, LEDs[led], state);	//write in the GPIO pin; turn 'on' and 'off' the LEDs.
}


/*Name: ledToggle
 *Description: toggles led in a pattern
 *Parameter: None
 *Return: None
 * */
void ledToggle(){

  static int l = 0 ; //holds LED position to turn on
  static int k = 7 ; //holds LED position to turn off
  static int onOffFlag = 1; //set the on and off pattern of LEDs


	// Turns on LEDs
	if(l < 8 && onOffFlag == 1) {
		writeLed(l, 1 );
	  l++;
	}
	// Turns off LEDs
	if( k >=0 && onOffFlag == 0){
		writeLed(k, 0);
	  k--;
	}

	//resetting led number
	if(l >= 8){
	  l = 0;
	  onOffFlag = 0;
	}
	if(k<0){
	  k = 7;
	  onOffFlag = 1;
	}
}


/*Name: dcStop
 *Description: Stops the motor
 *Parameter: None
 *Return: None
 * */
void dcStop( ){
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET); //disable the motor
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_RESET); //disable the motor
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
    return;
}


/*Name: dcInit
 *Description: Initializes peripherals for DC motor
 *Parameter: None
 *Return: None
 * */
void dcInit( ){
	__GPIOF_CLK_ENABLE();
	 /*Configure GPIO pins for PF2 and PF4*/
	GPIO_InitStruct.Pin = (GPIO_PIN_2 | GPIO_PIN_4);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}


/*Name: encoder
 *Description: run encoder to check the position of the motor and display on the screen
 *Parameter: None
 *Return: None
 * */
void encoder( ){
	uint32_t position = TIM3->CNT;
	char posStr[20];
	GotoXY(0,1);
	itoa(position, posStr, 10);
	PutStr(posStr);
	return;

}

/*Name: encoderInit
 *Description: Initializes peripherals for encoder
 *Parameter: None
 *Return: None
 * */
void encoderInit(){

	TIM_Encoder_InitTypeDef encoderConfig;

	__GPIOC_CLK_ENABLE();
	 /*Configure PC6 and PC7 for encoder*/
	GPIO_InitStruct.Pin = (GPIO_PIN_6 | GPIO_PIN_7);
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = 2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	/* initialize encoder data structure*/
	encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	encoderConfig.IC1Polarity = 0;
	encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC1Prescaler = 0;
	encoderConfig.IC1Filter = 3;
	encoderConfig.IC2Polarity = 0;
	encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC2Prescaler = 0;
	encoderConfig.IC2Filter = 3;

	rc = HAL_TIM_Encoder_Init(&htim3,&encoderConfig);
	if(rc != HAL_OK) {
		printf("Failed to initialize Timer 3 Encoder rc=%u\n",(unsigned)rc);
		return;
	}
	rc = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	if(rc != HAL_OK) {
		printf("Failed to start Timer 3 Encoder rc=%u\n",(unsigned)rc);
		return;
	}
	rc = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
	if(rc != HAL_OK) {
		printf("Failed to start Timer 3 Encoder rc=%u\n",(unsigned)rc);
		return;
	}

	TIM3->CNT = 0;
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  //TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  encoderInit();

  HAL_TIM_MspPostInit(&htim3);

}

// Name: TIM1_BRK_TIM15_IRQHandler
// Description: interrupt handler for timer 15
// Parameters: None
// Return: None
void TIM1_BRK_TIM15_IRQHandler(){
    HAL_TIM_IRQHandler(&htim15);  //reset the interrupt flag
    if(counter15 == 500 ){
        if (ledFlag == 1){
            	ledToggle();
         }

        counter15 = 0;
    }
    counter15++;
}


/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 35999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Timer interrupt intializer */
  HAL_NVIC_SetPriority(TIM15_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM15_IRQn);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
