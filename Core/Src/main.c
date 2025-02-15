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
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define U_TIMER LL_HRTIM_TIMER_B
#define V_TIMER LL_HRTIM_TIMER_F
#define W_TIMER LL_HRTIM_TIMER_C

#define GATE_DRIVER_RESET_US 1U
#define deadTime 640

// #define USE_EMRAX_MOTOR
#define USE_AMK_MOTOR

#ifdef USE_EMRAX_MOTOR
#define N_STEP_ENCODER 8192U
#define N_POLES 10
#endif
#ifdef USE_AMK_MOTOR
#define N_STEP_ENCODER 262144U
#define N_POLES 10U
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DELAY_US(us) \
  do { \
    uint32_t clk_cycle_start = DWT->CYCCNT;\
    uint32_t clk_cycles = (SystemCoreClock / 1000000U) * us;\
    while ((DWT->CYCCNT - clk_cycle_start) < clk_cycles);\
  } while (0)

#define ENDAT_DIR_WRITE LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)

#define ENDAT_DIR_Read LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)

// Gate driver ready and !fault signals
#define DRV_RDY_UH ((GPIOC->IDR&(1U<<11))!=0)
#define DRV_FLT_UH ((GPIOC->IDR&(1U<<10))!=0)
#define DRV_RDY_UL ((GPIOA->IDR&(1U<<15))!=0)
#define DRV_FLT_UL ((GPIOA->IDR&(1U<<12))!=0)
#define DRV_RDY_VH ((GPIOA->IDR&(1U<<8))!=0)
#define DRV_FLT_VH ((GPIOC->IDR&(1U<<9))!=0)
#define DRV_RDY_VL ((GPIOC->IDR&(1U<<8))!=0)
#define DRV_FLT_VL ((GPIOB->IDR&(1U<<15))!=0)
#define DRV_RDY_WH ((GPIOB->IDR&(1U<<11))!=0)
#define DRV_FLT_WH ((GPIOB->IDR&(1U))!=0)
#define DRV_RDY_WL ((GPIOB->IDR&(1U<<1))!=0)
#define DRV_FLT_WL ((GPIOB->IDR&(1U<<10))!=0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t adc_data[4];

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

char printBuffer[1024];

#ifdef USE_EMRAX_MOTOR
uint16_t Encoder_os = 731; // encoder offset angle
uint16_t KTY_LookupR[] = {980,1030,1135,1247,1367,1495,1630,1772,1922,2000,2080,2245,2417,2597,2785,2980,3182,3392,3607,3817,3915,4008,4166,4280};
int16_t KTY_LookupT[] = {-55,-50,-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,125,130,140,150};
uint16_t KTY_LookupSize = 24;
#endif
#ifdef USE_AMK_MOTOR
uint16_t Encoder_os = 0; // TODO
int16_t KTY_LookupR[] = {359,391,424,460,498,538,581,603,626,672,722,773,826,882,940,1000,1062,1127,1194,1262,1334,1407,1482,1560,1640,1722,1807,1893,1982,2073,2166,2261,2357,2452,2542,2624};
int16_t KTY_LookupT[] = {-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300};
uint16_t KTY_LookupSize = 36;
#endif

PID PID_I_d, PID_I_q;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void printCANBus(char* text);
void resetGateDriver();
void disableGateDriver();
void writePwm(uint32_t timer, int32_t duty);
int16_t lookupTbl(int16_t* source, int16_t* target);
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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Init DWT delay
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;  // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Enable microsecond counter
  LL_TIM_EnableCounter(TIM2);

  // encoder setup (Emrax motor)
  ENDAT_DIR_Read;
  LL_SPI_Enable(SPI1);

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
  // Motor variables
  uint32_t motor_PhysPosition;
  float motor_ElecPosition;
  float U_current, V_current, W_current;
  
  // Gate driver variables
  // format: |NA|NA|UH|UL|VH|VL|WH|WL|
  uint8_t driver_RDY = 0;
  uint8_t driver_OK = 0;

  // timing stuff
  uint32_t micros = TIM2->CNT;
  uint32_t lastMicros = 0;
  float dt = (micros - lastMicros) * 1e-6f;

  // requests
  float TargetCurrent = 1.0f;
  
  while (1)
  {
    LL_mDelay(100);
    micros = TIM2->CNT;
    dt = (micros - lastMicros) * 1e-6f;

    // convert ADC values to phase current and motor temperature
    U_current = (adc_data[0] - 2500) / 13.33f;
    V_current = (adc_data[1] - 2500) / 13.33f;
    W_current = (adc_data[2] - 2500) / 13.33f;
    int16_t KTY_Resistance = adc_data[3] >> 1;

    #ifdef USE_EMRAX_MOTOR
    // read motor position (RM44SI encoder)
    LL_SPI_TransmitData16(SPI1, 0);
    uint32_t startTime = TIM2->CNT;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
      if (TIM2->CNT - startTime > 10000U) break;
    }
    motor_PhysPosition = LL_SPI_ReceiveData16(SPI1);
    motor_PhysPosition = (motor_PhysPosition & 0x7FFF) >> 2;
    #endif
    #ifdef USE_AMK_MOTOR
    // read motor position (AMK type-P encoder)

    uint32_t data = 0U;
    uint8_t transferCount = 1U;
    uint32_t buf;
    uint8_t ones = 0xFEU;
    uint8_t bitCount = 0U;

    while (LL_SPI_IsActiveFlag_RXNE(SPI1)) LL_SPI_ReceiveData8(SPI1); // clear SPI buffer
    LL_SPI_Disable(SPI1);
    ENDAT_DIR_WRITE;
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_10BIT);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_Enable(SPI1);
    LL_SPI_TransmitData16(SPI1, 0b1000011101U); // send mode 1: Encoder send position values
    uint32_t startTime = TIM2->CNT;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)){
      if (TIM2->CNT - startTime > 20U) {
        printCANBus("timeout 1\n");
        break;
      }
    }
    LL_SPI_ReceiveData16(SPI1);
    ENDAT_DIR_Read;
    LL_SPI_Disable(SPI1);
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_Enable(SPI1);
    while (data == 0) { // wait for encoder to respond
      if (transferCount > 5) {
        //printCANBus("timeout 2t\n");
        data = 1U;
        break;
      }
      LL_SPI_TransmitData8(SPI1, 0U);
      startTime = TIM2->CNT;
      while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
        if (TIM2->CNT - startTime > 16U) {
          printCANBus("timeout 2");
          break;
        }
      }
      LL_SPI_ReceiveData8(SPI1);
      // sprintf(printBuffer, "data=%lu\n", data);
      // printCANBus(printBuffer);
      transferCount++;
    }
    while (data & ones) {
      bitCount++;
      ones = ones << 1;
    }

    buf = data << (25 - bitCount);
    LL_SPI_Disable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_10BIT);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_Enable(SPI1);
    LL_SPI_TransmitData16(SPI1, 0U);
    startTime = TIM2->CNT;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)){
      if (TIM2->CNT - startTime > 20U) {
        printCANBus("timeout 3\n");
        break;
      }
    }
    data = LL_SPI_ReceiveData16(SPI1);
    buf |= data << (15 - bitCount);

    LL_SPI_Disable(SPI1);
    LL_SPI_SetDataWidth(SPI1, (14 - bitCount) << 8);
    LL_SPI_SetRxFIFOThreshold(SPI1, (14 - bitCount < 9) ? LL_SPI_RX_FIFO_TH_QUARTER : LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_Enable(SPI1);
    if (15 - bitCount < 9) {
      LL_SPI_TransmitData8(SPI1, 0);
    }
    else {
      LL_SPI_TransmitData16(SPI1, 0);
    }
    startTime = TIM2->CNT;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)){
      if (TIM2->CNT - startTime > 32U) {
        printCANBus("timeout 4\n");
        break;
      }
    }
    data = LL_SPI_ReceiveData16(SPI1);
    buf |= data;

    motor_PhysPosition = (buf >> 5U) & (N_STEP_ENCODER-1);
    #endif


    motor_PhysPosition += Encoder_os;
    motor_ElecPosition = fmodf(motor_PhysPosition / N_STEP_ENCODER * N_POLES, 1.0f) * M_PI * 2.0f; // radians

    // read gate driver status (FLT and RDY pins)
    driver_RDY = (DRV_RDY_UH<<5) | (DRV_RDY_UL<<4) | (DRV_RDY_VH<<3) | (DRV_RDY_VL<<2) | (DRV_RDY_WH<<1) | DRV_RDY_WL;
    driver_OK  = (DRV_FLT_UH<<5) | (DRV_FLT_UL<<4) | (DRV_FLT_VH<<3) | (DRV_FLT_VL<<2) | (DRV_FLT_WH<<1) | DRV_FLT_WL;
    if ((driver_RDY | driver_OK) != 63){ // 0b00111111
      disableGateDriver();
    }

    // FOC
    float sin_elec_position = sinf(motor_ElecPosition);
    float cos_elec_position = cosf(motor_ElecPosition);
    // Clarke transform
    float I_a = U_current * 0.66666667f - V_current * 0.33333333f - W_current * 0.33333333f;
    float I_b = 0.5773502691896257f * (V_current - W_current);
    // Park transform
    float I_d = I_a * cos_elec_position + I_b * sin_elec_position;
    float I_q = I_b * cos_elec_position - I_a * sin_elec_position;
    // PI controllers on Q and D
    float cmd_d = PID_update(&PID_I_d, I_d, 0.0f, dt);
    float cmd_q = PID_update(&PID_I_q, I_q, TargetCurrent, dt);
    // Inverse Park transform
    float cmd_a = cmd_d * cos_elec_position - cmd_q * sin_elec_position;
    float cmd_b = cmd_q * cos_elec_position + cmd_d * sin_elec_position;
    // Inverse Clarke transform
    int16_t duty_u = cmd_a * 64000;
    int16_t duty_v = (cmd_a * -0.5f + 0.8660254037844386f * cmd_b) * 64000;
    int16_t duty_w = (cmd_a * -0.5f - 0.8660254037844386f * cmd_b) * 64000;
    // Update duty cycle
    writePwm(U_TIMER, duty_u);
    writePwm(V_TIMER, duty_v);
    writePwm(W_TIMER, duty_w);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    sprintf(printBuffer, "ADC1_1: %d, ADC1_2: %d, ADC1_3: %d, ADC2_5: %d, ", 
    adc_data[0], adc_data[1], adc_data[2], adc_data[3]);
    printCANBus(printBuffer);
    sprintf(printBuffer, "motor_PhysPosition: %lu\n", motor_PhysPosition);
    printCANBus(printBuffer);
    sprintf(printBuffer, "driver_RDY: %u driver_OK: %u\n", driver_RDY, driver_OK);
    printCANBus(printBuffer);
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
void resetGateDriver() {
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_12);
  DELAY_US(GATE_DRIVER_RESET_US);
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_12);
}

void disableGateDriver() {
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_12);
}

void printCANBus(char* text) {
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

void writePwm(uint32_t timer, int32_t duty) {
  // single sided
  if (duty < deadTime && duty > -deadTime) {
    duty = 0;
  }
  else if (duty > 64000 - deadTime) {
    duty = 64000;
  }
  else if (duty < deadTime - 64000) {
    duty = -64000;
  }

  if (duty > 0) {
    LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, duty);
    LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, 0);
  }
  else {
    LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, 0);
    LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, duty);
  }
  return;
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
