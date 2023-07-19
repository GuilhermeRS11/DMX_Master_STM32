/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

/* Macros para utilizar os mesmos pinos como UART e GPIO */
#define DMX_UART_Init() MX_USART2_UART_Init()
#define DMX_UART_DeInit HAL_UART_DeInit(&huart2)
#define DMX_GPIO_DeInit() HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2); // Desativa o modo GPIO
#define DMX_Set_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
#define DMX_Set_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
#define DMX_Set_DE_LOW() HAL_GPIO_WritePin(DMX_DE_GPIO_Port, DMX_DE_Pin, GPIO_PIN_RESET);
#define DMX_Set_DE_HIGH() HAL_GPIO_WritePin(DMX_DE_GPIO_Port, DMX_DE_Pin, GPIO_PIN_SET);

/* Private define ------------------------------------------------------------*/
#define TIMEOUT 100
#define NUM_CHANNELS 40
//#define DEBUG_RDM
//#define DEBUG_DMX
#define GUI_RDM_DMX

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void DMX_GPIO_Init(void);
void DMX_send_command(uint8_t* frame, uint16_t size);
void delay_us(uint32_t us);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	  HAL_Init();
	  SystemClock_Config();

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_USART1_UART_Init();
	  MX_TIM2_Init();
	  HAL_TIM_Base_Start(&htim2);


		#ifdef DEBUG_RDM
		uint8_t TN = 0x00;
		uint8_t ID = 0x01;
		uint8_t Identify_start_stop = 0x01;
		uint16_t PID_Resquested = 0x25FA;
		uint16_t DMX_address = 0xC13F;
		uint16_t Sub_Dev = 0x2AF7;
		uint64_t LB_PD = 0x0;
		uint64_t UB_PD = 0xFFFFFFFFFFFF;
		//uint64_t UID_D = 0x123456789ABC;
		uint64_t UID_D = 0xFFFFFFFFFFFF;
		uint64_t UID_S = 0xCBA987654321;

		uint8_t rxBuffer[MAX_BUFFER_SIZE];
		uint16_t rxDataLength = 0;

		unsigned char data_send_message[] = "\nData de Envio:\r\n"; //Data to send
		uint8_t data_received_message[40];

		/* Envio de dados ----------------------------------------------------------------------------------------------*/

		uint8_t* dmx_rdm_data = SET_identify_device(UID_D, UID_S, TN, ID, Sub_Dev, Identify_start_stop);
		//uint8_t* dmx_rdm_data = DISC_unique_branch(UID_D, UID_S, TN, ID, LB_PD, UB_PD);

		// Envia para a serial debug os dados a serem enviados
		unsigned char viewMessage[15];
		sprintf(viewMessage, "Frame RDM enviado: \r\n");

		for(int i = 0; i < dmx_rdm_data[2] + 2; i++){
			sprintf(viewMessage, "[%d] - 0x%02x\r\n", i, sizeof(dmx_rdm_data));
			HAL_UART_Transmit(&huart1, viewMessage, sizeof(viewMessage), TIMEOUT);
			//HAL_UART_Transmit(&huart2, 0b11001100, 1, TIMEOUT);
		}

		/* Recepção de dados -----------------------------------------------------------------------------------------------------------*/

		//HAL_UART_Receive(&huart2, data_received_message, 40, TIMEOUT);

		// Envia para a serial debug os dados lidos do RDM
		unsigned char data_receive_message[] = "\nData de Recebimento:\r\n";
		HAL_UART_Transmit(&huart1, data_receive_message, sizeof(data_receive_message), 10);	// Sending in normal mode
		/*for(int i = 0; i < data_received_message[2] + 2; i++){
			sprintf(viewMessage, "[%d] - 0x%02x\r\n", i, data_received_message[i]);
			HAL_UART_Transmit(&huart1, viewMessage, sizeof(viewMessage), TIMEOUT);
		}*/
		#endif

		#ifdef DEBUG_DMX
		uint8_t dmx_rdm_data[56] = {0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
														    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
														    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
														    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

		// Envia para a serial debug os dados a serem enviados
		unsigned char viewMessage[30];
		unsigned char viewDMX[30];

		sprintf(viewMessage, "Frame DMX enviado:\r\n");
		HAL_UART_Transmit(&huart1, viewMessage, sizeof(viewMessage), 10);	// Sending in normal mode
		for(int i = 0; i < sizeof(dmx_rdm_data); i++){
			sprintf(viewDMX, "[%d] - 0x%02x\r\n", i, dmx_rdm_data[i]);
			HAL_UART_Transmit(&huart1, viewDMX, sizeof(viewDMX), TIMEOUT);
		}

	  // Faz a transmissão via RS485
		DMX_send_command(dmx_rdm_data, sizeof(dmx_rdm_data));
		#endif

	  #ifdef GUI_RDM_DMX

		#define GUI_addr &huart1
		#define LIGHTING_addr &huart2

		uint8_t* receiveBuffer = NULL;
		uint8_t dataReceived;

		uint16_t receivedIndex = 0;
		uint8_t GUI_receiveFinished = 0;
		uint8_t GUI_receive = 1;

		uint32_t currentTime;

		DMX_UART_Init();

		while (1){
					/*
					 *
					 *
					 *
					 * */

					if(GUI_receive == 1){
						/* Recebe dados da GUI */
						if(HAL_UART_Receive (GUI_addr, &dataReceived, 1, 20) == HAL_OK){

							uint8_t* tempBuffer = (uint8_t*)realloc(receiveBuffer, (receivedIndex + 1) * sizeof(uint8_t)); /* Buffer temporario para alocacao dinamica*/
							receiveBuffer = tempBuffer;
							receiveBuffer[receivedIndex++] = dataReceived;
							GUI_receiveFinished = 1; /* Avisa que quando acabar o recebimento de bytes, pode enviar para a luminaria*/

						} else if(GUI_receiveFinished == 1){
							/* Se acabou o recebimento, envia para a luminária e reseta os parametros de recebimento*/
							DMX_send_command(receiveBuffer, receivedIndex-1); // AQUI PRECISA SER receivedIndex - 1, MAS NAO TA INDO

							if(receiveBuffer[0] == 0xCC){	// Se for um frame DMX, a proxima iteracao sera a espera de um comando vindo da luminaria
								GUI_receive = 0; /* Entra para a secao que espera o recebimento de dados da luminaria e envia para a GUI*/
								currentTime = __HAL_TIM_GET_COUNTER(&htim2); /* Inicia timer para definir rota de retorno para este modo*/
							}

							GUI_receiveFinished = 0;
							receivedIndex = 0;
							free(receiveBuffer);
							receiveBuffer = NULL;
						}
					} else{
						/* Recebe dados da luminaria */
						if(HAL_UART_Receive (LIGHTING_addr, &dataReceived, 1, 20) == HAL_OK){
							uint8_t* tempBuffer = (uint8_t*)realloc(receiveBuffer, (receivedIndex + 1) * sizeof(uint8_t)); /* Buffer temporario para alocacao dinamica*/
							receiveBuffer = tempBuffer;
							receiveBuffer[receivedIndex++] = dataReceived;
							GUI_receiveFinished = 1; /* Avisa que quando acabar o recebimento de bytes, pode enviar para a luminaria*/

						} else if(GUI_receiveFinished == 1){
							/* Se acabou o recebimento, envia para a GUI e reseta os parametros de recebimento*/
							HAL_UART_Transmit(GUI_addr, receiveBuffer, receivedIndex-1, TIMEOUT);

							GUI_receiveFinished = 0;
							receivedIndex = 0;
							GUI_receive = 1; /* Volta para o recebimento de dados da GUI e envio para a luminaria*/
							free(receiveBuffer);
							receiveBuffer = NULL;

						} else if((currentTime - __HAL_TIM_GET_COUNTER(&htim2)) > 50000){
							GUI_receive = 1; /* Volta para o recebimento de dados da GUI e envio para a luminaria*/
						}
					}
		}
		#endif
}

/*
 * Função que envia o comando DMX seguindo os tempos de MBB, break e MAB exigidos pela norma
 *
 * */
void DMX_send_command(uint8_t* frame, uint16_t size){
	DMX_GPIO_Init();   // Inicia DMX modo GPIO
	//delay_us(2000); 	 // Delay para começar a comunicar

	DMX_Set_DE_HIGH(); // Habilita o barramento DMX para escrita (Necessidade do RS485)

	DMX_Set_HIGH();		 // Seta o MBB
	delay_us(50);

	DMX_Set_LOW(); 		 // Seta o Break
	delay_us(250);

	// O Time after break é implementado pela UART, através do idle frame

	DMX_GPIO_DeInit(); // Desativa o modo GPIO
	DMX_UART_Init();// Inicia novamente o modo USART
	HAL_UART_Transmit(LIGHTING_addr, frame, size, TIMEOUT);

	DMX_Set_DE_LOW(); // Desabilita o barramento DMX para escrita (Necessidade do RS485)
	DMX_UART_DeInit;
}

static void DMX_GPIO_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

	// Configure GPIO pin as output
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void delay_us(uint32_t us){
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 250000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DMX_DE_GPIO_Port, DMX_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DMX_DE_Pin */
  GPIO_InitStruct.Pin = DMX_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DMX_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
