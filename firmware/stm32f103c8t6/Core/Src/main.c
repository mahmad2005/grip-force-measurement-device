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
//#define ROW_CS_PIN GPIO_PIN_12
//#define COL1_CS_PIN GPIO_PIN_13
//#define COL2_CS_PIN GPIO_PIN_14

#define WR_PIN GPIO_PIN_10
#define EN_PIN GPIO_PIN_1
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
#define CON_SEL_L_4 GPIO_PIN_1		// PORTA

#define ROW_SEL_L GPIO_PIN_8
#define ROW_SEL_R GPIO_PIN_9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Updated 2D array to store pressure readings
uint16_t pressure_readings[32][64];

// Start and end markers for data transmission
uint8_t start_marker[] = {0xAA, 0xBB};
uint8_t end_marker[] = {0xBB, 0xAA};

// Updated 2D array to store pressure readings
//uint16_t pressure_readings[32][64];
uint16_t dummy_receive[32][64];


// globals
static uint16_t bufA[32][64];
static uint16_t bufB[32][64];
static volatile uint16_t (*tx_buf)[64]   = bufA;  // currently served to SPI
static volatile uint16_t (*fill_buf)[64] = bufB;  // being filled by ADC
static volatile uint8_t frame_ready = 0;


volatile uint8_t data_ready = 0;
volatile uint8_t spi_busy = 0;

volatile uint8_t need_scan  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
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
    GPIO_InitStruct.Pin = CON_SEL_R_1 | CON_SEL_R_2 | CON_SEL_R_3 | CON_SEL_L_1 | CON_SEL_L_2 | CON_SEL_L_3 | ROW_SEL_L | ROW_SEL_R;
    HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

    // Configure chip select pins
	GPIO_InitStruct.Pin = CON_SEL_R_4 | CON_SEL_L_4;
	HAL_GPIO_Init(GPIO_PORT_A, &GPIO_InitStruct);

    // Configure WR and EN pins
    GPIO_InitStruct.Pin = WR_PIN | EN_PIN;
    HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

    // Set default states
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_R_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_L_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_SET);
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
//uint8_t MapColumnIndex(uint8_t col) {
//    if (col < 16) {
//        return col; // 0–15 map directly to 0–15
//    } else if (col < 32) {
//        return 31 - (col - 16); // 16–31 map in reverse to 31–16
//    } else if (col < 48) {
//        return col; // 32–47 map directly to 32–47
//    } else {
//        return 63 - (col - 48); // 48–63 map in reverse to 63–48
//    }
//}

uint8_t MapColumnIndex(uint8_t col) {
    if (col < 16) {
        return 16 + col; // 0–15 map directly to 16–31
    } else if (col < 32) {
        return (31-col); // 16–31 map in reverse to 15–0
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
void ADG732_SelectChannel(uint8_t col_channel, uint8_t row_channel_val) {
    // Select row
	uint8_t row_channel;
	if (row_channel_val < 16) {
		//row_channel = 15 - row_channel_val;
		row_channel = row_channel_val;
	}
	else {
		 row_channel = 16 + (31 - row_channel_val);
		//row_channel = row_channel_val;
	}

//	HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIO_PORT, EN_PIN, GPIO_PIN_SET);   // disable (EN is active-low)

	if (row_channel >= 0 && row_channel <8 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
	}
	else if (row_channel >= 0 && row_channel <8 && col_channel >= 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel >= 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}
//bottom half
	else if (row_channel >= 16 && row_channel <24 && col_channel < 32) {
			HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
	}
	else if (row_channel >= 16 && row_channel <24 && col_channel >= 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}
	else if (row_channel >= 24 && row_channel <32 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_RESET);
	}

//	if (col_channel <32) {
//		if (row_channel_val < 16) {
//			//row_channel = 15 - row_channel_val;
//			row_channel_val = 16 + row_channel_val;
//		}
//		else {
//			row_channel_val = 31- row_channel_val;
//			//row_channel = row_channel_val;
//		}
//	}
//
//    ADG732_SetAddress(row_channel_val);
	uint8_t row_address = row_channel_val;
	if (col_channel < 32) {
	    if (row_channel_val < 16) {
	        row_address = 16 + row_channel_val;
	    } else {
	        row_address = 31 - row_channel_val;
	    }
	}
	ADG732_SetAddress(row_address);

    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_SET);

//    if (col_channel <32 && row_channel_val < 16) {
//    	row_channel = (31 - row_channel_val);
//    }
	if (col_channel <32) {
		if (row_channel_val < 16) {
			//row_channel = 15 - row_channel_val;
			//row_channel = 31 - row_channel_val;
			//row_channel = 31- row_channel_val;
		}
	}


    //Select Column
//    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_RESET);

    ADG732_SetAddress(col_channel);
    delay_us(1);

	if (row_channel >= 0 && row_channel <8 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_RESET);
	}
	else if (row_channel >= 0 && row_channel <8 && col_channel >= 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_1, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_2, GPIO_PIN_RESET);
	}
	else if (row_channel >= 8 && row_channel <16 && col_channel >= 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_2, GPIO_PIN_RESET);
	}
//bottom half
	else if (row_channel >= 16 && row_channel <24 && col_channel < 32) {
			HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_3, GPIO_PIN_RESET);
	}
	else if (row_channel >= 16 && row_channel <24 && col_channel >= 32) {
		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_3, GPIO_PIN_RESET);
	}
	else if (row_channel >= 24 && row_channel <32 && col_channel < 32) {
		HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_L_4, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_R_4, GPIO_PIN_RESET);
	}

    //HAL_GPIO_WritePin(GPIO_PORT, ROW_CS_PIN, GPIO_PIN_RESET);
//    ADG732_SetAddress(col_channel);
//    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_L_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_R_4, GPIO_PIN_SET);

//	if (row_channel >= 0 && row_channel <8 && col_channel >= 32) {
//		HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_RESET);
//	}

//    HAL_GPIO_WritePin(GPIO_PORT, EN_PIN, GPIO_PIN_RESET); // enable
//    delay_us(2);
}

// Helper: compute the actual row address for the ADG732 given the logical row and which column half we're on
static inline uint8_t MapRowAddrForSide(uint8_t row_logical, uint8_t col_channel) {
    // Your stated rules:
    // - If col < 32:
    //     - if row < 16: row_addr = 16 + row
    //     - else       : row_addr = 31 - row
    // - If col >= 32:
    //     - if row < 16: row_addr = row (unchanged)
    //     - else       : row_addr = 16 + (31 - row)

    if (col_channel < 32) {
        if (row_logical < 16) {
            return (uint8_t)(16 + row_logical);
        } else {
            return (uint8_t)(31 - row_logical);
        }
    } else {
        if (row_logical < 16) {
            return row_logical;
        } else {
            return (uint8_t)(16 + (31 - row_logical));
        }
    }
}


// Select a specific row and column
void ADG732_SelectChannel2(uint8_t col_channel /* already mapped */, uint8_t row_logical /* 0..31 raw */) {
    // ----- Decide side and row group (for enables) without mutating inputs -----
    const uint8_t side_left = (col_channel < 32);
    const uint8_t row_group = row_logical >> 3;   // 0..3 (groups of 8 rows)

    // ----- Drive ROW side enable first -----
    // Deassert both rows, then assert the one we want
    HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, side_left ? ROW_SEL_L : ROW_SEL_R, GPIO_PIN_RESET);

    // ----- Set ROW address (mapped for this side), strobe WR -----
    const uint8_t row_address = MapRowAddrForSide(row_logical, col_channel);
    ADG732_SetAddress(row_address);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_SET);

    // Optionally release row enables if your hardware needs it:
     HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_L, GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIO_PORT, ROW_SEL_R, GPIO_PIN_SET);

    // ----- Now set COLUMN address, then select the correct COLUMN enable for this side+row_group -----
    ADG732_SetAddress(col_channel);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_PORT, WR_PIN, GPIO_PIN_SET);

    // Deassert all column enables first
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_L_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_R_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_R_4, GPIO_PIN_SET);

    // Assert only the one we need
    if (side_left) {
        switch (row_group) {
            case 0: HAL_GPIO_WritePin(GPIO_PORT,   CON_SEL_L_1, GPIO_PIN_RESET); break;
            case 1: HAL_GPIO_WritePin(GPIO_PORT,   CON_SEL_L_2, GPIO_PIN_RESET); break;
            case 2: HAL_GPIO_WritePin(GPIO_PORT,   CON_SEL_L_3, GPIO_PIN_RESET); break;
            default: HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_L_4, GPIO_PIN_RESET); break;
        }
    } else {
        switch (row_group) {
            case 0: HAL_GPIO_WritePin(GPIO_PORT,   CON_SEL_R_1, GPIO_PIN_RESET); break;
            case 1: HAL_GPIO_WritePin(GPIO_PORT,   CON_SEL_R_2, GPIO_PIN_RESET); break;
            case 2: HAL_GPIO_WritePin(GPIO_PORT,   CON_SEL_R_3, GPIO_PIN_RESET); break;
            default: HAL_GPIO_WritePin(GPIO_PORT_A, CON_SEL_R_4, GPIO_PIN_RESET); break;
        }
    }

    // If you want to latch columns and then release enables immediately:
    // HAL_GPIO_WritePin(GPIO_PORT, CON_SEL_L_1, GPIO_PIN_SET); ...
    // (or keep asserted during the ADC read—depends on your timing design)
}


void inside_for_loop(uint8_t row) {
	for (uint8_t col = 0; col < 64; col++) {
		uint8_t mapped_col = MapColumnIndex(col); // Map logical to physical column
		uint8_t mapped_row = MapRowIndex(row);
		ADG732_SelectChannel2(mapped_col, row);
		delay_us(DELAY_MicroSec);
		//delay_us(10);

		// Read ADC value and store in the array
		//pressure_readings[row][col] = Read_ADC_Channel0();

		//pressure_readings[row][col] = (row >= 24) ? Read_ADC_Channel0() : 0;
		uint16_t adc_val;
		Read_ADC_Channel0();
		 adc_val = Read_ADC_Channel0();  // Always read

		pressure_readings[row][col] = adc_val; //1024;
	}
}



// Collect pressure readings
void Collect_Pressure_Readings(void) {
    for (uint8_t row = 0; row < 32; row++) {			// 0 to 32
    inside_for_loop(row);

    }
}


static void collect_into(uint16_t dst[32][64]) {
    for (uint8_t r=0; r<32; r++) {
        for (uint8_t c=0; c<64; c++) {
            // select row/col
        	uint8_t mapped_col = MapColumnIndex(c); // Map logical to physical column
        	ADG732_SelectChannel2(mapped_col, r);
            // settle
        	delay_us(DELAY_MicroSec);
            // read ADC
        	uint16_t adc_val;
			Read_ADC_Channel0();
			adc_val = Read_ADC_Channel0();  // Always read
            dst[r][c] = adc_val;
        }
    }
}



// Collect pressure readings
void Collect_Dummy_Pressure_Readings(void) {
    for (uint8_t row = 0; row < 32; row++) {			// 0 to 32
        for (uint8_t col = 0; col < 64; col++) {	// 0 to 64
            // Read ADC value and store in the array
            pressure_readings[row][col] = 1048;//Read_ADC_Channel0();
            if (row >= 31 && col >= 60){
            	pressure_readings[row][col] = 2122;
            }
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  UD_GPIO_Init();
  DWT_Init();
  ADG732_Init();
  ADC1_Init();
  USART1_Init();


  HAL_Delay(1000);
  myPinToggle();




  //Collect_Dummy_Pressure_Readings();
  Collect_Pressure_Readings();
  HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&pressure_readings, (uint8_t *)&dummy_receive, sizeof(pressure_readings));
  // arm first transfer
  //HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)tx_buf, (uint8_t*)dummy_receive, sizeof(bufA));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // Fill the background buffer while SPI can serve the current one
//      collect_into((uint16_t (*)[64])fill_buf);   // your 32x64 scan
//      __DMB();                                    // memory barrier
//      frame_ready = 1;                            // publish new frame
      // (optional small delay)

	  //Collect_Pressure_Readings();    // Collect readings from the 8x8 matrix
	  //Transmit_Pressure_Readings_RowChunks();

	  if (need_scan == 1) {
		  Collect_Pressure_Readings();

		  need_scan = 0;
	  }

	  HAL_Delay(1);  // Delay between each full scan to avoid overflow
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//    if (hspi->Instance == SPI1) {
//        //HAL_UART_Transmit(&huart1, (uint8_t *)"Request Received: ", 18, HAL_MAX_DELAY);
//        //HAL_UART_Transmit(&huart1, rx_buffer, SPI_MSG_LEN, HAL_MAX_DELAY);
//        //HAL_UART_Transmit(&huart1, (uint8_t *)&dummy_receive[1][1], sizeof(dummy_receive[1][1]), HAL_MAX_DELAY);
//        //HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
//
//        //char msg[32];
//        //sprintf(msg, "Value: %u\r\n", dummy_receive[1][1]);
//        //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//        // re-arm the transfer
//        //HAL_SPI_TransmitReceive_IT(&hspi1, slave_response, rx_buffer, SPI_MSG_LEN);
//        //HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)pressure_readings, (uint8_t *)dummy_receive, sizeof(pressure_readings));
//        //HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&pressure_readings[1][1], (uint8_t *)&dummy_receive[1][1], sizeof(pressure_readings[1][1]));
//    	//Collect_Pressure_Readings();
//    	//Collect_Dummy_Pressure_Readings();
//        HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&pressure_readings, (uint8_t *)&dummy_receive, sizeof(pressure_readings));
//        Collect_Pressure_Readings();
//    }
//}

//// ----- callback (VERY SHORT) -----
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//    if (hspi->Instance == SPI1) {
//        if (frame_ready) {                  // a new frame is ready?
//            // swap buffers
//            uint16_t (*tmp)[64] = (uint16_t (*)[64])tx_buf;
//            tx_buf   = fill_buf;
//            fill_buf = tmp;
//            frame_ready = 0;
//        }
//        // re-arm SPI immediately with the current tx_buf
//        HAL_SPI_TransmitReceive_IT(&hspi1,
//            (uint8_t*)tx_buf, (uint8_t*)dummy_receive, sizeof(bufA));
//    }
//}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        HAL_SPI_TransmitReceive_IT(&hspi1,
            (uint8_t*)&pressure_readings, (uint8_t*)&dummy_receive,
            sizeof(pressure_readings));                 // re-arm immediately
        // now do NOT block here; kick a flag for the main loop to scan
        need_scan = 1;
    }
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
