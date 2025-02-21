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
#include "svpwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define PI 3.14159265358979323846f
#define PI_2o3 2.0943951023931953f

#define U_TIMER LL_HRTIM_TIMER_B
#define V_TIMER LL_HRTIM_TIMER_F
#define W_TIMER LL_HRTIM_TIMER_C

#define GATE_DRIVER_RESET_US 1U
#define deadTime 640

// #define USE_EMRAX_MOTOR
#define USE_AMK_MOTOR

#ifdef USE_EMRAX_MOTOR
#define N_STEP_ENCODER 8192U
#define N_POLES 10U
#endif
#ifdef USE_AMK_MOTOR
#define N_STEP_ENCODER 262144UL
#define N_POLES 5U
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
uint32_t Encoder_os = 731; // encoder offset angle
int32_t KTY_LookupR[] = {980,1030,1135,1247,1367,1495,1630,1772,1922,2000,2080,2245,2417,2597,2785,2980,3182,3392,3607,3817,3915,4008,4166,4280};
int32_t KTY_LookupT[] = {-55,-50,-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,125,130,140,150};
uint16_t KTY_LookupSize = 24;
#endif
#ifdef USE_AMK_MOTOR
uint32_t Encoder_os = 678; // TODO: automatically measure this
int32_t KTY_LookupR[] = {359,391,424,460,498,538,581,603,626,672,722,773,826,882,940,1000,1062,1127,1194,1262,1334,1407,1482,1560,1640,1722,1807,1893,1982,2073,2166,2261,2357,2452,2542,2624};
int32_t KTY_LookupT[] = {-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300};
uint16_t KTY_LookupSize = 36;
#endif

PID PID_I_d, PID_I_q;

tSVPWM sSVPWM = SVPWM_DEFAULTS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void printCANBus(char* text);
void resetGateDriver();
void disableGateDriver();
void writePwm(uint32_t timer, int32_t duty);
int32_t lookupTbl(const int32_t* source, const int32_t* target, const uint32_t size, const int32_t value);
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

  // Enable TIM1, TIM15, and TIM5 (MOSFET temperture PWM signal)
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_EnableCounter(TIM15);
  LL_TIM_EnableCounter(TIM5);

  // Enable HRTIM (gate drive signals)
  writePwm(U_TIMER, 0);
  writePwm(V_TIMER, 0);
  writePwm(W_TIMER, 0);
  LL_HRTIM_EnableOutput(HRTIM1, 
  LL_HRTIM_OUTPUT_TB1|LL_HRTIM_OUTPUT_TB2|
  LL_HRTIM_OUTPUT_TF1|LL_HRTIM_OUTPUT_TF2|
  LL_HRTIM_OUTPUT_TC1|LL_HRTIM_OUTPUT_TC2);
  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_MASTER|U_TIMER|V_TIMER|W_TIMER);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Motor variables
  int32_t motor_PhysPosition;
  float motor_ElecPosition;
  float U_current, V_current, W_current;
  PID_setParams(&PID_I_q, 0.0f, 0.003f, 0.0f);
  PID_setParams(&PID_I_d, 0.0f, 0.003f, 0.0f);
  sSVPWM.enInType = AlBe;  // set the input type
  sSVPWM.fUdc = 1.0f;    // set the DC-Link voltage in Volts
  sSVPWM.fUdcCCRval = 64000; // set the Max value of counter compare register which equal to DC-Link voltage
  int32_t KTY_Temperature;

  // Gate driver variables
  // format: |NA|NA|UH|UL|VH|VL|WH|WL|
  uint8_t driver_RDY = 0;
  uint8_t driver_OK = 0;
  resetGateDriver();

  // timing stuff
  uint32_t micros = TIM2->CNT;
  uint32_t lastMicros = 0;
  float dt = (micros - lastMicros) * 1e-6f;

  // requests
  float TargetCurrent = 2.0f;

  // measure ADC offset
  int16_t adc_os[3] = {0, 0, 0};
  for (int i = 0; i < 8; i++) {
    LL_mDelay(1);
    adc_os[0] += adc_data[0];
    adc_os[1] += adc_data[1];
    adc_os[2] += adc_data[2];
  }
  adc_os[0] = adc_os[0] >> 3;
  adc_os[1] = adc_os[1] >> 3;
  adc_os[2] = adc_os[2] >> 3;
  
  while (1)
  {
    //LL_mDelay(20);
    lastMicros = micros;
    micros = TIM2->CNT;
    dt = (micros - lastMicros) * 1e-6f;


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
    uint32_t buf = 0U;
    ENDAT_DIR_WRITE;
    while (LL_SPI_IsActiveFlag_RXNE(SPI1)) LL_SPI_ReceiveData8(SPI1); // clear SPI buffer
    LL_SPI_Disable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_10BIT);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
    LL_SPI_Enable(SPI1);
    LL_SPI_TransmitData16(SPI1, 0b0000011100U); // send mode 1: Encoder send position values
    uint32_t startTime = TIM2->CNT;
    // This seems very messy but I have to do this to save time
    // Do math while waiting for SPI to complete
    // convert ADC values to phase current and motor temperature
    U_current = -(adc_data[0] - adc_os[0]) / 13.33f;
    V_current = -(adc_data[1] - adc_os[1]) / 13.33f;
    W_current = -(adc_data[2] - adc_os[2]) / 13.33f;
    KTY_Temperature = lookupTbl(KTY_LookupR, KTY_LookupT, KTY_LookupSize, adc_data[3] >> 1);
    while (LL_SPI_IsActiveFlag_BSY(SPI1)){
      if (TIM2->CNT - startTime > 20U) {
        printCANBus("timeout 1\n");
        break;
      }
    }
    LL_SPI_ReceiveData16(SPI1);
    ENDAT_DIR_Read;
    LL_SPI_Disable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
    LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_LSB_FIRST);
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
    LL_SPI_Enable(SPI1);
    SPI1->DR = 0;
    SPI1->DR = 0;
    // read gate driver status (FLT and RDY pins)
    driver_RDY = (DRV_RDY_UH<<5) | (DRV_RDY_UL<<4) | (DRV_RDY_VH<<3) | (DRV_RDY_VL<<2) | (DRV_RDY_WH<<1) | DRV_RDY_WL;
    driver_OK  = (DRV_FLT_UH<<5) | (DRV_FLT_UL<<4) | (DRV_FLT_VH<<3) | (DRV_FLT_VL<<2) | (DRV_FLT_WH<<1) | DRV_FLT_WL;
    if ((driver_RDY & driver_OK) != 63){ // 0b00111111
      disableGateDriver();
    }
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    buf |= LL_SPI_ReceiveData16(SPI1) >> 8;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    buf |= ((uint32_t)LL_SPI_ReceiveData16(SPI1)) << 8;
    motor_PhysPosition = N_STEP_ENCODER-1 - (buf & (N_STEP_ENCODER - 1));
    #endif


    motor_ElecPosition = fmodf((float)(motor_PhysPosition + Encoder_os) / N_STEP_ENCODER * N_POLES, 1.0f) * PI * 2.0f; // radians
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
    float cmd_d = PID_update(&PID_I_d, I_d, 0.0f, dt)*0.5;
    float cmd_q = PID_update(&PID_I_q, I_q, TargetCurrent, dt);
    // Inverse Park transform
    float cmd_a = cmd_d * cos_elec_position - cmd_q * sin_elec_position;
    float cmd_b = cmd_q * cos_elec_position + cmd_d * sin_elec_position;
    // Inverse Clarke transform
    // SVPWM_DutyCycles duty_cycles;
    // calculate_SVPWM(cmd_a, cmd_b, &duty_cycles);
    sSVPWM.fUal = cmd_a;	// set a new value of voltage Alpha
    sSVPWM.fUbe = cmd_b;	// set a new value of voltage Beta
    sSVPWM.m_calc(&sSVPWM);		// call the SVPWM duty cycles calculation function
    // Update duty cycle
    int32_t duty_u = sSVPWM.fCCRA;
    int32_t duty_v = sSVPWM.fCCRB;
    int32_t duty_w = sSVPWM.fCCRC;
    writePwm(U_TIMER, duty_u);
    writePwm(V_TIMER, duty_v);
    writePwm(W_TIMER, duty_w);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // printCANBus("----------------------\n");
    // sprintf(printBuffer, "ADC1_1: %d, ADC1_2: %d, ADC1_3: %d, ADC2_5: %d\n", 
    // adc_data[0], adc_data[1], adc_data[2], adc_data[3]);
    // printCANBus(printBuffer);
    // sprintf(printBuffer, "motor_PhysPosition: %6ld motor_ElecPosition: %.03f\n", motor_PhysPosition, motor_ElecPosition);
    // printCANBus(printBuffer);
    // sprintf(printBuffer, "driver_RDY: %u driver_OK: %u\n", driver_RDY, driver_OK);
    // printCANBus(printBuffer);
    // sprintf(printBuffer, "U: %6ld, V: %6ld, W: %6ld, I_d: %.03f, cmd_d: %.03f, I_q: %.03f, cmd_q: %.03f\n", duty_u, duty_v, duty_w, I_d, cmd_d, I_q, cmd_q);
    // printCANBus(printBuffer);
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
  uint32_t duty_H, duty_L;
  if (duty <= deadTime) {
    duty_H = 0;
    duty_L = 0;
  }
  else if (duty >= 64000 - deadTime) {
    duty_H = 64001;
    duty_L = 64001;
  }
  else {
    if (duty <= deadTime << 1) {
      duty_H = 0;
      duty_L = duty + deadTime;
    }
    else if (duty >= 64000 - (deadTime << 1)) {
      duty_H = duty - deadTime;
      duty_L = 64001;
    }
    else {
      duty_H = duty - deadTime;
      duty_L = duty + deadTime;
    }
  }
  LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, duty_H);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, duty_L);
}

int32_t lookupTbl(const int32_t* source, const int32_t* target, const uint32_t size, const int32_t value) {
  uint32_t left = 0;
  uint32_t right = size - 1;
  uint32_t middle = (left + right) >> 1;
  while (right - left > 1) {
    if (source[middle] < value) {
      left = middle;
    }
    else if (source[middle] > value){
      right = middle;
    }
    else {
      return target[middle];
    }
    middle = (left + right) >> 1;
  }
  return (value - source[left]) * (target[right] - target[left]) / (source[right] - source[left]) + target[left];
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
