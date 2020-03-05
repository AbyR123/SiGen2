/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */

volatile uint8_t txPending = 0; 			// flaga informujaca o trwajacej transmisji bajtu
// volatile uint8_t framePending = 0;		// ranmka oczekująca

/* 
volatile uint8_t frameStage = 0;		// stage - etap
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
	  // zapisz odebrany bajt do bufora kołowego
	  RingBuffer_write(&recieve_buffer, read_b);
	  HAL_UART_Receive_IT(&huart2, &read_b, 1);
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//	__HAL_UART_CLEAR_FEFLAG(&huart2);					// Tutaj powinno być czyszczenie flagi, jeżeli HAL tego nie zrobił
  if (huart->Instance == USART2)
  {
	// jeśli są bajty w buforze kołowym do wysłania to wyślij jeden
	if( !RingBuffer_isEmpty(&transmit_buffer) ){
		
		write_b = RingBuffer_read( &transmit_buffer );
		HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(&huart2, &write_b, 1);
		if(ret == HAL_ERROR){
			transmit_buffer.read_cursor--;
		}
    }
  }
}

/**
	Wysyłanie stringa na usarta, używając bufora kołowego.
 */
void send_string(char* data) {
	// kopiowanie do bufora kołowego danc=ych do wysłania
	int i=0;
	int len = strlen(data);
	// pętla będzie się kręcić w kółko aż cały string nie zostanie wepchnięty do bufora kołowego
	while ( i < len ) {
		// jeśli bufor kołowy jest pełen to pomiń tę operację i spróbuj za chwilę
		if(!RingBuffer_isFull( &transmit_buffer )){
			RingBuffer_write( &transmit_buffer, data[i] );
			i++;
		}
		__disable_irq(); // wyłączenie globalnych przerwań
		// jeśli bufor kołowy nie jest pusty oraz flaga TXE jest ustawiona to zainicjuj wysyłanie
		if(!RingBuffer_isEmpty( &transmit_buffer ){
			if((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET) ) {	// podane warunki są potwierdzeniem, 
																							// że transmisja trwa ustawiona
		//		if(!RingBuffer_isEmpty( &transmit_buffer)){				// w przypadku podania pustego String-a
					// pobranie z bufora pierwszego znaku do wysłania
					write_b = RingBuffer_read( &transmit_buffer );
					// wysłanie pierwszego znaku
					HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(&huart2, &write_b, 1);
					if(ret == HAL_ERROR){
						transmit_buffer.read_cursor--;
					}
		//		}
			}
		}
		__enable_irq(); // włączenie globalnych przerwań
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// stan odbierania ramki
enum frame_state_t{
	  RAMKA_NOT_STARTED,
	  RAMKA_STARTED,
	  RAMKA_FINISHED
};

void przetworz_ramke(char * ramka, int dlugosc){

	// format ramki:  <forma>,<czestotliwosc>,<amplituda>,<faza>
	// suma kontrolna: Liczba zapisana w formie tekstu.
	// Jest sumą wszystkich znaków w przetworzonych elementach.
	ramka[dlugosc]=0;

	// zamień przecinki na 0 i wyszukaj stringi
	char * elements[16]; // tablica stringów stworzona z podzielonej ramki
	int elements_cursor=0;
	elements[elements_cursor]=ramka+0;
	elements_cursor++;
	int i;
	for(i=0;i<dlugosc;i++){
		// czy znak jest przecinkiem
		if(ramka[i]==','){
			ramka[i]=0; // zamien przecinek na 0
			elements[elements_cursor] = ramka+i+1; // miejsce za 0 to kolejny string
			elements_cursor++;
		}
	}

	int N = elements_cursor; // ilosc znalezionych elementow

	// odrzuć ramkę jeśli któryś z elementów jest pusty
	//(string o zerowej dlugosci - pierwszy bajt tego stringa wynosi 0)
	for(i=0;i<N;i++){
		if(elements[i][0] == 0)
			return;
	}


	if(N==4) // wymagana ilosc elementow w ramce
	{


		char waveform = elements[0][0]; // pierwszy bajt pierwszego stringa

		float czestotliwosc = atof(elements[1]); // string do float
		float amplituda = atof(elements[2]);
		float faza = atof(elements[3]);

		char komunikat[64];
		sprintf(komunikat,"\r\n  Ksztalt fali: %c\r\n", waveform);
		send_string(komunikat);

		sprintf(komunikat,"Czestotliwosc: %f\r\n", czestotliwosc);
		send_string(komunikat);
		sprintf(komunikat,"Amplituda: %f\r\n", amplituda);
		send_string(komunikat);
		sprintf(komunikat,"Faza: %f\r\n", faza);
		send_string(komunikat);



	}


	//return;

}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  RingBuffer_init(&recieve_buffer);
  HAL_UART_Receive_IT( &huart2, &read_b, 1 );
  /* USER CODE END 2 */
 
  // tablica przechwoujaca ramkę
  const int DLUGOSC_RAMKI=64;
  char ramka[DLUGOSC_RAMKI+1];

  // stan odbierania ramki
  enum frame_state_t frame_state=RAMKA_NOT_STARTED;
  int frame_read_cursor;
  /* Infinite loop */
  while (1)
  {
	  	// sprawdź czy przyszedł bajt i o go odczytaj
		if(!RingBuffer_isEmpty(&recieve_buffer)){
			// odczytaj bajt
			unsigned char value = RingBuffer_read(&recieve_buffer);

			// przetwórz bajt w zależności od stanu
			switch(frame_state){
			case RAMKA_FINISHED:
			case RAMKA_NOT_STARTED:
				// szukaj początku ramki
				if(value==':'){
					frame_state = RAMKA_STARTED;
					frame_read_cursor=0;
				}
				break;
			case RAMKA_STARTED:
				if(value==';')
				{
					// zakończ zbieranie ramki
					frame_state = RAMKA_FINISHED;
				}
				else if(value==':'){
					// odrzuć poprzednią ramkę i zacznij zbieranie nowej
					frame_read_cursor=0;
				}
				else{
					if(frame_read_cursor < DLUGOSC_RAMKI){
						ramka[frame_read_cursor] = value;
						frame_read_cursor++;
					}
					else{
						// przepełnienie ramki
						frame_state = RAMKA_NOT_STARTED;
					}
				}
				break;
			}
		}

		// przetwarzanie odebranej ramki
		if(frame_state == RAMKA_FINISHED) {
			przetworz_ramke(ramka, frame_read_cursor);
			frame_state = RAMKA_NOT_STARTED;
		}
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
