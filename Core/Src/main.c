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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
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
#define N_STEP_ENCODER 8192
#define N_POLES 10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_data[4];

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

char printBuffer[1024];

uint16_t Encoder_os = 731; // encoder offset angle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void print_CANBus(char* text);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN2_Init();
  MX_HRTIM1_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Enable microsecond counter
  LL_TIM_EnableCounter(TIM5);

  // encoder setup (Emrax motor)
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4); // data port input mode

  // Enable ADC1 (phase current sensors) with DMA
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_data);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 3);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_REG_StartConversion(ADC1);

  // Enable ADC2 (motor temperature sensor) with DMA
  LL_ADC_Enable(ADC2);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC2));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&ADC2->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&adc_data[3]);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_ADC_REG_StartConversion(ADC2);

  // CANbus setup
  HAL_FDCAN_Start(&hfdcan2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LL_mDelay(100);
    sprintf(printBuffer, "ADC1_1: %d, ADC1_2: %d, ADC1_3: %d, ADC2_5: %d, ", 
    adc_data[0], adc_data[1], adc_data[2], adc_data[3]);
    print_CANBus(printBuffer);

    // read motor position
    uint16_t phys_position;
    float elec_position;
    uint8_t TxBuffer[2];
    uint8_t RxBuffer[2];
    HAL_SPI_TransmitReceive(&hspi1, TxBuffer, RxBuffer, 1, 5000);
    memcpy(&phys_position, RxBuffer, 2);
    phys_position = (phys_position & 0x7FFF) >> 2;
    sprintf(printBuffer, "phys_position: %d\n", phys_position);
    print_CANBus(printBuffer);
    phys_position += Encoder_os;
    elec_position = fmodf(phys_position / N_STEP_ENCODER * N_POLES, 1.0f) * M_PI * 2.0f; // radians
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 20, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(160000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void print_CANBus(char* text) {
  TxHeader.Identifier = 0x3FF;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0);
  uint32_t i = 0;
  while (text[i] != '\0') {
    TxData[i & 7U] = text[i];
    if ((i & 7U) == 7U) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
      while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0);
    }
    i++;
  }
  if (i & 7U) {
    for (uint8_t j = i & 7U; j < 8; j++) {
      TxData[j] = 0;
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
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
