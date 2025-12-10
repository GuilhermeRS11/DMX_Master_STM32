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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DMX.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT 100
#define NUM_CHANNELS 40
#define MINIMAL_BREAK_TIME 80 //80us (por norma o minimo o 88us)
#define MINIMAL_MAB_TIME 8 //8us (por norma o minimo o 12us)
//#define DEBUG_RDM
//#define DEBUG_DMX
#define GUI_RDM_DMX
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Macros para utilizar os mesmos pinos como UART e GPIO */
#define DMX_UART_Init() MX_USART2_UART_Init()
#define DMX_UART_DeInit HAL_UART_DeInit(&huart2)
#define DMX_GPIO_DeInit() HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2); // Desativa o modo GPIO
#define DMX_Set_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
#define DMX_Set_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
#define DMX_Set_DE_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
#define DMX_Set_DE_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

#define HEADER_BUFFER_SIZE 9
#define GUI_addr &huart1
#define LIGHTING_addr &huart2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// Defina os estados da maquina de estados para o envio DMX
typedef enum {
    STATE_IDLE,
    STATE_MBB,
    STATE_BREAK,
    STATE_MAB,
    STATE_DATA,
} DMX_State;

// Variaveis globais
DMX_State dmx_state = STATE_IDLE;
uint16_t dmx_index = 0;
uint16_t receivedIndex = 0;
uint8_t* receiveBuffer = NULL;

typedef struct {
    uint8_t *data;
    uint16_t capacity;
} DynamicFrame;

DynamicFrame currentFrame = {0};
uint8_t header_sequence[] = {0x7E, 0x06, 0x3A};
uint8_t tail_sequence[] = {0x7E, 0x06, 0x3B};
uint8_t TBB_value = 0;
uint8_t TBF_value = 0;
uint8_t break_time_value = 88;
uint8_t countinuous_DMX_send = 0;
uint8_t uartBuffer[HEADER_BUFFER_SIZE];
uint8_t* DMX_buffer_toSend = NULL;
uint16_t DMX_buffer_toSend_Size = 0;
uint8_t process_frame = 0;
uint8_t data_already_send = 1;
uint16_t Frame_Size = 0;
uint16_t UART2_rxBuffer_Size = 0;
uint8_t UART2_on_Start = 0;
uint8_t UART2_rxBuffer[1024];
uint8_t UART2_Begin_rxBuffer[3];
uint8_t on_MAB = 0;
uint8_t on_break_mark = 0;
uint8_t data_sent;
uint8_t gui_frame_is_rdm = 0;
uint16_t gui_frame_size = 0;
uint8_t gui_rdm_header_bytes[4] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
static void DMX_GPIO_Init(void);
static void DMX_GPIO_ToUSART(void);
static void DMX_ScheduleTimer(uint16_t period_us);
static uint16_t DMX_GetMarkBeforeBreak(void);
static uint16_t DMX_GetMarkAfterBreak(void);
static uint16_t DMX_GetBreakTimeUs(void);
static void DMX_DisableTransmitter(void);
static void DMX_EnableTransmitter(void);
void DMX_SendHandler(void);
void startTiming(void);
void stopTiming(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == GUI_addr) {
        TIM2->CNT = 0; // Zera o timer que reseta uart em eventos de travamento

        // Sempre que esse callback dispara, significa que o cabecalho de 9 bytes
        // foi recebido via DMA em uartBuffer. Vamos validar e ler o payload.
        if ((uartBuffer[0] == header_sequence[0]) &&
            (uartBuffer[1] == header_sequence[1]) &&
            (uartBuffer[2] == header_sequence[2])) {

            uint16_t capacity = (uint16_t)((uartBuffer[3] << 8) | uartBuffer[4]);

            // Detecta se é frame RDM (start code 0xCC logo após o tamanho)
            if (uartBuffer[5] == 0xCCU) {
                gui_frame_is_rdm = 1U;
            } else {
                gui_frame_is_rdm = 0U;
                // Para DMX puro, atualiza parametros de tempo
                TBB_value = uartBuffer[5];
                TBF_value = uartBuffer[6];
                break_time_value = uartBuffer[7];
                countinuous_DMX_send = uartBuffer[8];
            }

            currentFrame.capacity = capacity;

            // Garante que nao ficamos com lixo antigo
            free(currentFrame.data);
            currentFrame.data = (uint8_t *)malloc(currentFrame.capacity);

            if (currentFrame.data != NULL) {
                // Le o restante do frame (dados + tail) byte a byte.
                // Para RDM, os primeiros 4 bytes do frame (0xCC,0x01,Message_Length e 1o byte do corpo)
                // ja foram recebidos junto com o cabecalho da interface em uartBuffer[5..8].
                // Copiamos esses 4 bytes para o inicio de currentFrame.data.
                uint16_t received = 0;
                if (gui_frame_is_rdm) {
                    currentFrame.data[0] = uartBuffer[5];
                    currentFrame.data[1] = uartBuffer[6];
                    currentFrame.data[2] = uartBuffer[7];
                    currentFrame.data[3] = uartBuffer[8];
                    received = 4;
                }

                while (received < currentFrame.capacity) {
                    if (HAL_UART_Receive(huart,
                                         &currentFrame.data[received],
                                         1,
                                         TIMEOUT) != HAL_OK) {
                        // Timeout ou erro ao ler este byte -> para
                        break;
                    }
                    received++;

                    if (received >= 3 &&
                        currentFrame.data[received - 3] == tail_sequence[0] &&
                        currentFrame.data[received - 2] == tail_sequence[1] &&
                        currentFrame.data[received - 1] == tail_sequence[2]) {
                        // Encontrou o tail completo
                        break;
                    }
                }

                if (received >= 3 &&
                    currentFrame.data[received - 3] == tail_sequence[0] &&
                    currentFrame.data[received - 2] == tail_sequence[1] &&
                    currentFrame.data[received - 1] == tail_sequence[2]) {
                    // Frame completo pronto para envio DMX/RDM
                    currentFrame.capacity = received;  // atualiza para o tamanho real lido
                    process_frame = 1;
                } else {
                    // Erro/timeout ou tail nao encontrado: descarta frame
                    free(currentFrame.data);
                    currentFrame.data = NULL;
                }
            }

            // Prepara para receber um novo cabecalho de 9 bytes via DMA
            HAL_UART_Receive_DMA(huart, uartBuffer, HEADER_BUFFER_SIZE);

        } else {
            // Cabecalho invalido, descarta e volta a esperar cabecalho
            free(currentFrame.data);
            currentFrame.data = NULL;
            HAL_UART_Receive_DMA(huart, uartBuffer, HEADER_BUFFER_SIZE);
        }
    }

    if (huart == LIGHTING_addr){

      /*
      // Corresponde a UART da interface RS485

      // Recebe os primeiros 3 bytes do frame.
      // Se for DMX espera receber 256 bytes
      // Se for RDM espera receber o tamanho designado para o frame 

      if(huart->ErrorCode != 0){
        HAL_UART_DeInit(huart);
        HAL_UART_Init(huart);
        #ifdef DEBUG_PYSICAL
          Debug_write_string("\n\rOcorreu um erro no recebimento e os dados nao serao processados");
        #endif

      } else {

        if (UART2_on_Start){
          // Se a UART receceu os 3 bytes de inicio, entao testa se o RDM ou DMX
          //e prapara o recebimento do restante dos dados
          
          

          UART2_on_Start = 0;

          if (UART2_Begin_rxBuffer[0] == 0xCC){
            uint16_t Frame_Size = UART2_rxBuffer_Size - 3;
            HAL_UART_Receive_IT(huart, UART2_rxBuffer, Frame_Size);

          } else{
            // Tratamento quando o dado recebido nao o DMX nem RDM
            HAL_NVIC_EnableIRQ(EXTI0_1_IRQn); //Libera para receber um novo frame
          }

        } else {
          HAL_NVIC_EnableIRQ(EXTI0_1_IRQn); //Libera para receber um novo frame
          // Envia o frame para a interface grafica

        }
      }*/
    }
}

// Funcao para lidar com a transmissao concluoda
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2){
		 // Este callback sera chamado quando a transmissao for concluoda
		DMX_Set_DE_LOW();  		 // Desabilitar o barramento DMX para escrita (Necessidade do RS485)
		dmx_state = STATE_IDLE;  // Transicao para o estado de IDLE
		data_sent = 1;

		// Libera o buffer de recebimento de dados
		if (!countinuous_DMX_send){
			DMX_buffer_toSend_Size = 0;
			free(DMX_buffer_toSend);
			DMX_buffer_toSend = NULL;
			data_already_send = 1; // Flag para indicar que os dados foram enviados
		}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  #define GUI_addr &huart1
  #define LIGHTING_addr &huart2

  uint8_t dataReceived;
  uint8_t GUI_receiveFinished = 0;
  uint8_t GUI_receive = 1;
  uint32_t currentTime;

  unsigned char viewDMX[20];

  DMX_UART_Init();
  HAL_TIM_Base_Start(&htim17);
  HAL_TIM_Base_Start_IT(&htim2);

  // Inicializa a DMA para a recepcao UART
  HAL_UART_Receive_DMA(&huart1, uartBuffer, HEADER_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	if(process_frame == 0 && &huart1.Lock == HAL_UNLOCKED){
//		HAL_UART_Receive_DMA(&huart1, uartBuffer, HEADER_BUFFER_SIZE);
//	}

	if(process_frame && (data_already_send || data_sent)){
		// Libera a memoria anteriormente alocada se necessario
		DMX_buffer_toSend_Size = 0;
		free(DMX_buffer_toSend);
		DMX_buffer_toSend = NULL;
  
		if (DMX_buffer_toSend != NULL)
			free(DMX_buffer_toSend);
  
		// Define o payload a ser enviado (DMX puro ou RDM)
		if (gui_frame_is_rdm && currentFrame.capacity > 3) {
			// currentFrame.data contem o frame RDM completo vindo da GUI
			// seguido do tail da interface (0x7E 0x06 0x3B).
			// Remove apenas o tail e envia o frame exatamente como a GUI montou.
			uint16_t rdm_total = (uint16_t)(currentFrame.capacity - 3); // descarta tail 0x7E 0x06 0x3B

			DMX_buffer_toSend_Size = rdm_total;
			DMX_buffer_toSend = (uint8_t *)malloc(DMX_buffer_toSend_Size);
			if (DMX_buffer_toSend != NULL) {
				memcpy(DMX_buffer_toSend, currentFrame.data, rdm_total);
			}
		} else {
			// Frame DMX: mantém comportamento original
			DMX_buffer_toSend_Size = currentFrame.capacity - 5;
			DMX_buffer_toSend = (DMX_buffer_toSend_Size > 0)
			                    ? (uint8_t *)malloc(DMX_buffer_toSend_Size)
			                    : NULL;
  
			if (DMX_buffer_toSend != NULL) {
				// Copia payload DMX sem os 5 bytes de controle/tail
				memcpy(DMX_buffer_toSend, currentFrame.data, DMX_buffer_toSend_Size);
			}
		}
  
		// Verifica se a alocacao de memoria foi bem-sucedida antes de iniciar o envio
		if (DMX_buffer_toSend != NULL && DMX_buffer_toSend_Size > 0) {
			dmx_state = STATE_MBB;
			data_already_send = 0;
			DMX_SendHandler();
		} else {
			// Lida com a falha na alocacao de memoria, se necessario
		}

		process_frame = 0;
		data_sent = 0;
		data_already_send = 0;

    	// Prepara nova recepcao
    	free(currentFrame.data);
    	currentFrame.data = NULL;
//    	HAL_UART_Receive_DMA(&huart1, uartBuffer, HEADER_BUFFER_SIZE);

	}

	// Verifica se deve enviar continuamente os dados
	if(countinuous_DMX_send && data_sent){
		data_sent = 0;
		dmx_state = STATE_MBB;
		DMX_SendHandler();
	}

//	if(GUI_receive == 1){
//		/* Recebe dados da GUI */
//
//		if(HAL_UART_Receive (GUI_addr, &dataReceived, 1, 2) == HAL_OK){
//			uint8_t* tempBuffer = (uint8_t*)realloc(receiveBuffer, (receivedIndex + 1) * sizeof(uint8_t)); /* Buffer temporario para alocacao dinamica*/
//			receiveBuffer = tempBuffer;
//			receiveBuffer[receivedIndex++] = dataReceived;
//			GUI_receiveFinished = 1; /* Avisa que quando acabar o recebimento de bytes, pode enviar para a luminaria*/
//
//		} else if(GUI_receiveFinished == 1){
//
//			/* Se acabou o recebimento, envia para a luminaria e reseta os parametros de recebimento*/
//			if(receivedIndex > 5){ //Verifica se ha dados para serem repassados
//				dmx_state = STATE_MBB;
//				DMX_SendHandler();
//			}
//
//			if(receiveBuffer[0] == 0xCC){	// Se for um frame RDM, a proxima iteracao sera a espera de um comando vindo da luminaria
//				GUI_receive = 0; /* Entra para a secao que espera o recebimento de dados da luminaria e envia para a GUI*/
//				currentTime = __HAL_TIM_GET_COUNTER(&htim2); /* Inicia timer para definir rota de retorno para este modo*/
//
//			}
//
//			GUI_receiveFinished = 0;
////			receivedIndex = 0;
////			free(receiveBuffer);
////			receiveBuffer = NULL;
//		}
//	} else{
//		/* Recebe dados da luminaria */
//		if(HAL_UART_Receive (LIGHTING_addr, &dataReceived, 1, 1) == HAL_OK){
//			uint8_t* tempBuffer = (uint8_t*)realloc(receiveBuffer, (receivedIndex + 1) * sizeof(uint8_t)); /* Buffer temporario para alocacao dinamica*/
//			receiveBuffer = tempBuffer;
//			receiveBuffer[receivedIndex++] = dataReceived;
//			GUI_receiveFinished = 1; /* Avisa que quando acabar o recebimento de bytes, pode enviar para a luminaria*/
//
//		} else if(GUI_receiveFinished == 1){
//			/* Se acabou o recebimento, envia para a GUI e reseta os parametros de recebimento*/
//			if(receivedIndex > 1)//Verifica se ha dados para serem repassados
//				HAL_UART_Transmit(GUI_addr, receiveBuffer, receivedIndex, 1);
//
//			GUI_receiveFinished = 0;
//			receivedIndex = 0;
//			GUI_receive = 1; /* Volta para o recebimento de dados da GUI e envio para a luminaria*/
//			free(receiveBuffer);
//			receiveBuffer = NULL;
//
//		} else if((currentTime - __HAL_TIM_GET_COUNTER(&htim2)) > TIME_WAIT_RDM_RESPONSE){
//			GUI_receive = 1; /* Volta para o recebimento de dados da GUI e envio para a luminaria*/
//		}
//	}
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
  htim2.Init.Period = 100000;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 48-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0xffff;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.BaudRate = 500000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Timing_test_Pin|LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DMX_DE_GPIO_Port, DMX_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Timing_test_Pin */
  GPIO_InitStruct.Pin = Timing_test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Timing_test_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Break_detection_Pin */
  GPIO_InitStruct.Pin = Break_detection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Break_detection_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DMX_DE_Pin */
  GPIO_InitStruct.Pin = DMX_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
/*
 * Funcao que envia o comando DMX seguindo os tempos de MBB, break e MAB exigidos pela norma
 *
 * */

static void DMX_GPIO_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	DMX_Set_HIGH();
}

static void DMX_GPIO_ToUSART(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void DMX_ScheduleTimer(uint16_t period_us){
	if (period_us == 0U){
		period_us = 1U;
	}
	__HAL_TIM_CLEAR_FLAG(&htim17, TIM_IT_UPDATE);
	TIM17->CNT = 0;
	TIM17->ARR = period_us;
	__HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);
}

static uint16_t DMX_GetMarkBeforeBreak(void){
	uint16_t value = TIME_BEFORE_BREAK;
	if (TBB_value != 0U){
		value = TBB_value;
	}
	if (value < MINIMAL_MAB_TIME){
		value = MINIMAL_MAB_TIME;
	}
	return value;
}

static uint16_t DMX_GetMarkAfterBreak(void){
	uint16_t value = TIME_AFTER_BREAK;
	if (TBF_value != 0U){
		value = TBF_value;
	}
	if (value < MINIMAL_MAB_TIME){
		value = MINIMAL_MAB_TIME;
	}
	return value;
}

static uint16_t DMX_GetBreakTimeUs(void){
	uint16_t value = break_time_value;
	if (value < MINIMAL_BREAK_TIME){
		value = MINIMAL_BREAK_TIME;
	}
	if (value > 13U){
		value -= 13U;
	}
	if (value == 0U){
		value = MINIMAL_BREAK_TIME;
	}
	return value;
}

static void DMX_DisableTransmitter(void){
	UART_HandleTypeDef *uart = LIGHTING_addr;
	while (__HAL_UART_GET_FLAG(uart, UART_FLAG_TC) == RESET){
	}
	CLEAR_BIT(uart->Instance->CR1, USART_CR1_TE);
}

static void DMX_EnableTransmitter(void){
	UART_HandleTypeDef *uart = LIGHTING_addr;
	SET_BIT(uart->Instance->CR1, USART_CR1_TE);
}

void DMX_SendHandler(void) {
    switch (dmx_state) {
        case STATE_IDLE:
            break;

        case STATE_MBB:
        	//DMX_Set_DE_HIGH(); DE retirado daqui pois o restante esta muito lento. Para poder sincronizar o acionamento de DE com MBB, foi colocado antes de break
            DMX_DisableTransmitter();
            DMX_GPIO_Init();
            DMX_Set_HIGH();
            DMX_ScheduleTimer(DMX_GetMarkBeforeBreak());
            dmx_state = STATE_BREAK;
            break;

        case STATE_BREAK:
            DMX_Set_LOW();
            DMX_Set_DE_HIGH(); // Via de regra nao deveria estar aqui, porem foi necessario para poder sincronizar subida com MBB (rever em outros micros)
            DMX_ScheduleTimer(DMX_GetBreakTimeUs());
            dmx_state = STATE_MAB;
            break;

        case STATE_MAB:
            DMX_Set_HIGH();
            DMX_ScheduleTimer(DMX_GetMarkAfterBreak());
            dmx_state = STATE_DATA;
            break;

        case STATE_DATA:
            DMX_Set_HIGH();
            DMX_GPIO_ToUSART();
            DMX_EnableTransmitter();
            dmx_state = STATE_IDLE;
            HAL_UART_Transmit_IT(LIGHTING_addr, DMX_buffer_toSend, DMX_buffer_toSend_Size);
            HAL_NVIC_EnableIRQ(TIM2_IRQn);
            break;
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == Break_detection_Pin){
		uint8_t currentButtonState = HAL_GPIO_ReadPin(Break_detection_GPIO_Port, Break_detection_Pin);
		if(currentButtonState == 0){ // Borda de 1 - 0 (Detecta o break)
			if(on_break_mark){
				TIM14->CNT = 0;
				UART2_on_Start = 1;
				//HAL_GPIO_TogglePin(GPIOC, LD4_Pin);

			} else if (on_MAB) {
				//Confere se o break 00o pelo menos 80us
				//uint32_t tempo = TIM14->CNT;
				if(TIM14->CNT > MINIMAL_BREAK_TIME && UART2_on_Start){
					//HAL_GPIO_TogglePin(GPIOC, LD4_Pin);
					// Avalia se o tempo de brak foi atingido. Se sim, passa para avaliacao do MAB
					on_break_mark = 0;
					on_MAB = 1;
					TIM14->CNT = 0;
				}
			}

		} else{						 // Borda de 0 - 1 (Confere se o break 00o pelo menos 80us)
			//uint32_t tempo = TIM14->CNT;
			if(TIM14->CNT > MINIMAL_BREAK_TIME && UART2_on_Start){
				//HAL_GPIO_TogglePin(GPIOC, LD4_Pin);
				HAL_NVIC_DisableIRQ(EXTI0_1_IRQn); //Trava a recepcao de sinais que nao sejam break

				HAL_UART_Init(LIGHTING_addr);
				HAL_UART_Receive_DMA(LIGHTING_addr, UART2_Begin_rxBuffer, 3);

			}
		}
	}
}

// Verifica se ocorreu um erro no recebimento dos dados que pode travar a recepcao
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM14) {  // Verifica se o callback o para o timer TIM14
		HAL_NVIC_EnableIRQ(EXTI0_1_IRQn); //Libera para receber um novo frame
		on_break_mark = 1;
		on_MAB = 0;
	}
}

// Funcao para iniciar a temporizacao
void startTiming() {
    HAL_GPIO_WritePin(GPIOC, Timing_test_Pin, GPIO_PIN_SET); // Defina o pino para alto
}

// Funcao para parar a temporizacao
void stopTiming() {
    HAL_GPIO_WritePin(GPIOC, Timing_test_Pin, GPIO_PIN_RESET); // Defina o pino para baixo
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
