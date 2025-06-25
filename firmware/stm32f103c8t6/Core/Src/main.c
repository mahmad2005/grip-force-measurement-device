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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define A0_PIN GPIO_PIN_5
//#define A1_PIN GPIO_PIN_6
//#define A2_PIN GPIO_PIN_7
//#define A3_PIN GPIO_PIN_8
//#define A4_PIN GPIO_PIN_9
#define ROW_CS_PIN GPIO_PIN_12
#define COL1_CS_PIN GPIO_PIN_13
#define COL2_CS_PIN GPIO_PIN_14
#define WR_PIN GPIO_PIN_15
#define EN_PIN GPIO_PIN_4
#define GPIO_PORT GPIOB

#define DELAY_MicroSec 10

#define GPIO_PORT_A GPIOA
#define GPIO_PORT_B GPIOB

#define A0_PIN GPIO_PIN_4
#define A1_PIN GPIO_PIN_3
#define A2_PIN GPIO_PIN_15
#define A3_PIN GPIO_PIN_14
#define A4_PIN GPIO_PIN_13

#define CON_SEL_R_1 GPIO_PIN_0
#define CON_SEL_R_2 GPIO_PIN_5
#define CON_SEL_R_3 GPIO_PIN_7
#define CON_SEL_R_4 GPIO_PIN_2		// PORTA

#define CON_SEL_L_1 GPIO_PIN_11
#define CON_SEL_L_2 GPIO_PIN_12
#define CON_SEL_L_3 GPIO_PIN_6
#define CON_SEL_L_4 GPIO_PIN_11		// PORTA

#define ROW_SEL_L GPIO_PIN_8
#define ROW_SEL_R GPIO_PIN_9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Updated 2D array to store pressure readings
uint16_t pressure_readings[32][64];

// Start and end markers for data transmission
uint8_t start_marker[] = {0xAA, 0xBB};
uint8_t end_marker[] = {0xBB, 0xAA};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void ADC1_Init(void);
void USART1_Init(void);

void DWT_Init(void);
void delay_us(uint32_t us);

uint16_t Read_ADC_Channel0(void);
void USART1_Transmit(const char *data);

void ADG732_Init(void);
void ADG732_SetAddress(uint8_t address);
uint8_t MapColumnIndex(uint8_t col);
void ADG732_SelectChannel(uint8_t col_channel, uint8_t row_channel);
void Collect_Pressure_Readings(void);
void Transmit_Pressure_Readings(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Initialize GPIO pins for ADG732 control
// Initialize the DWT for microsecond delays
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Microsecond delay using DWT
void delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

// GPIO Initialization Function
void UD_GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();  // Enable GPIOC clock
    //__HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-Pull Output
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // No Pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Low-speed
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


// Initialize ADC1 on Channel 0
void ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();

    ADC_HandleTypeDef hadc1;
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

// Read ADC value from Channel 0
uint16_t Read_ADC_Channel0(void) {
    ADC_HandleTypeDef hadc1;
    hadc1.Instance = ADC1;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adc_value;
}

// Initialize USART1 for UART communication
void USART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 921600;//115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

// Function to transmit data over USART1
void USART1_Transmit(const char *data) {
    HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
}

// Initialize GPIO pins for ADG732
void ADG732_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure address pins (A0–A4)
    GPIO_InitStruct.Pin = A0_PIN | A1_PIN | A2_PIN | A3_PIN | A4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

    // Configure chip select pins
    GPIO_InitStruct.Pin = ROW_CS_PIN | COL1_CS_PIN | COL2_CS_PIN;
    HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

    // Configure WR and EN pins
    GPIO_InitStruct.Pin = WR_PIN | EN_PIN;
    HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

    // Set default states
    HAL_GPIO_WritePin(GPIO_PORT, ROW_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, COL1_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, COL2_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, EN_PIN, GPIO_PIN_RESET);
}

// Set multiplexer address using address pins A0–A4
void ADG732_SetAddress(uint8_t address) {
    HAL_GPIO_WritePin(GPIO_PORT, A0_PIN, (address & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, A1_PIN, (address & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, A2_PIN, (address & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, A3_PIN, (address & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, A4_PIN, (address & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Map logical column index to physical column index
uint8_t MapColumnIndex(uint8_t col) {
    if (col < 16) {
        return col; // 0–15 map directly to 0–15
    } else if (col < 32) {
        return 31 - (col - 16); // 16–31 map in reverse to 31–16
    } else if (col < 48) {
        return col; // 32–47 map directly to 32–47
    } else {
        return 63 - (col - 48); // 48–63 map in reverse to 63–48
    }
}

uint8_t MapRowIndex(uint8_t row) {
	if (row <16) {
		return row; // 0–15 map directly to 0–15
	} else {
		return 31 -(row - 16); // 16–31 map in reverse to 31–16
	}
}

// Select a specific row and column
void ADG732_SelectChannel(uint8_t col_channel, uint8_t row_channel) {
    // Select row
	//HAL_GPIO_WritePin(GPIO_PORT, COL1_CS_PIN, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIO_PORT, COL2_CS_PIN, GPIO_PIN_RESET);
	//delay_us(10);
	if (row_channel >= 0 && row_channel <8 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_1, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_2, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_2, GPIO_PIN_RESET);
	}
	else if (row_channel >= 16 && row_channel <24 && col_channel < 32) {
			HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_3, GPIO_PIN_RESET);
	}
	else if (row_channel >= 16 && row_channel <24 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_3, GPIO_PIN_RESET);
	}
	else if (row_channel >= 24 && row_channel <32 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_L_4, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_R_4, GPIO_PIN_RESET);
	}
    //HAL_GPIO_WritePin(GPIO_PORT, ROW_CS_PIN, GPIO_PIN_RESET);
    ADG732_SetAddress(row_channel);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, ROW_CS_PIN, GPIO_PIN_SET);
    //delay_us(10); // 5 MicroSec delay to clear the current on the pad

    // Select column multiplexer
//    if (col_channel < 32) {
//        HAL_GPIO_WritePin(GPIO_PORT, COL1_CS_PIN, GPIO_PIN_RESET);
//    } else {
//        col_channel -= 32;
//        HAL_GPIO_WritePin(GPIO_PORT, COL2_CS_PIN, GPIO_PIN_RESET);
//    }
	if (row_channel >= 0 && row_channel <8 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, COL1_CS_PIN, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}
	else if (row_channel >= 16 && row_channel <24 && col_channel < 32) {
			HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
	}
	else if (row_channel >= 16 && row_channel <24 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}
	else if (row_channel >= 24 && row_channel <32 && col_channel > 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}
    ADG732_SetAddress(col_channel);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, COL1_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, COL2_CS_PIN, GPIO_PIN_SET);
}



// Collect pressure readings
void Collect_Pressure_Readings(void) {
    for (uint8_t row = 0; row < 32; row++) {			// 0 to 32
        for (uint8_t col = 0; col < 64; col++) {	// 0 to 64
            uint8_t mapped_col = MapColumnIndex(col); // Map logical to physical column
            uint8_t mapped_row = MapRowIndex(row);
            ADG732_SelectChannel(mapped_col, mapped_row);
            delay_us(DELAY_MicroSec);
            //delay_us(100);

            // Read ADC value and store in the array
            pressure_readings[row][col] = Read_ADC_Channel0();
        }
    }
}

// Transmit pressure readings over UART
void Transmit_Pressure_Readings(void) {
    char buffer[50];
    for (uint8_t row = 0; row < 32; row++) {
        for (uint8_t col = 0; col < 64; col++) {
            snprintf(buffer, sizeof(buffer), "Row: %d, Col: %d, ADC: %d\r\n", row, col, pressure_readings[row][col]);
            USART1_Transmit(buffer);
        }
    }
}

// Transmit entire at once
void Transmit_Pressure_Readings_atOnce(void) {
    // Transmit start marker
    HAL_UART_Transmit(&huart1, start_marker, sizeof(start_marker), HAL_MAX_DELAY);

    // Transmit the entire matrix in one call
    HAL_UART_Transmit(&huart1, (uint8_t*)pressure_readings, sizeof(pressure_readings), HAL_MAX_DELAY);

    // Transmit end marker
    HAL_UART_Transmit(&huart1, end_marker, sizeof(end_marker), HAL_MAX_DELAY);
}


// Transmit in Row Chunks
void Transmit_Pressure_Readings_RowChunks(void) {
    // Transmit start marker
    HAL_UART_Transmit(&huart1, start_marker, sizeof(start_marker), HAL_MAX_DELAY);

    // Transmit each row as a chunk
    for (uint8_t row = 0; row < 32; row++) {
        HAL_UART_Transmit(&huart1, (uint8_t*)pressure_readings[row], sizeof(pressure_readings[row]), HAL_MAX_DELAY);
    }

    // Transmit end marker
    HAL_UART_Transmit(&huart1, end_marker, sizeof(end_marker), HAL_MAX_DELAY);
}

void myPinToggle(void){
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	// Delay for 500ms
	HAL_Delay(500);
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  UD_GPIO_Init();
  DWT_Init();
  ADG732_Init();
  ADC1_Init();
  USART1_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Collect_Pressure_Readings();    // Collect readings from the 8x8 matrix
	  Transmit_Pressure_Readings_RowChunks();

	  HAL_Delay(10);  // Delay between each full scan to avoid overflow
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
