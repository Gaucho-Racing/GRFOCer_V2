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
#include <stdbool.h>
#include <stdlib.h>
#include "arm_math.h"
#include "defines.h"
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

/* USER CODE BEGIN PV */
volatile int16_t adc_data[4];

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

char printBuffer[1024];

// Motor variables
volatile bool SPI_Wait = false;
volatile uint8_t SPI_WaitState = 0;
volatile uint32_t SPI_buf = 0U;
volatile int32_t motor_PhysPosition;
int32_t motor_lastPhysPosition;
volatile uint32_t motor_lastMeasTime;
volatile float motor_ElecPosition;
volatile float U_current, V_current, W_current;
volatile int32_t motor_speed = 0; // encoder LSBs / second
int32_t KTY_Temperature;
#ifdef USE_EMRAX_MOTOR
volatile int32_t Encoder_os = 731; // encoder offset angle
int32_t KTY_LookupR[] = {980,1030,1135,1247,1367,1495,1630,1772,1922,2000,2080,2245,2417,2597,2785,2980,3182,3392,3607,3817,3915,4008,4166,4280};
int32_t KTY_LookupT[] = {-55,-50,-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,125,130,140,150};
uint16_t KTY_LookupSize = 24;
#endif
#ifdef USE_AMK_MOTOR
volatile int32_t Encoder_os = 3277;
int32_t KTY_LookupR[] = {359,391,424,460,498,538,581,603,626,672,722,773,826,882,940,1000,1062,1127,1194,1262,1334,1407,1482,1560,1640,1722,1807,1893,1982,2073,2166,2261,2357,2452,2542,2624};
int32_t KTY_LookupT[] = {-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300};
uint16_t KTY_LookupSize = 36;
#endif

// FOC variables
volatile float sin_elec_position, cos_elec_position;
volatile float I_a, I_b, I_q, I_d;
volatile float cmd_q = 0.0f, cmd_d = 0.0f;
volatile float cmd_a = 0.0f, cmd_b = 0.0f;
volatile float integ_q = 0.0f, integ_d = 0.0f;
volatile float Kp_Iq = 0.0f, Ki_Iq = 0.1f;
volatile float Kp_Id = 0.0f, Ki_Id = 0.1f;
volatile float I_d_err, I_q_err;
volatile float TargetCurrent = 0.0f;
volatile float TargetFieldWk = 0.0f;
volatile const uint8_t SVPWM_PermuataionMatrix[6][3] = {	
	{ 1, 2, 0 },
	{ 3, 1, 0 },
	{ 0, 1, 2 },
	{ 0, 3, 1 },
	{ 2, 0, 1 },
	{ 1, 0, 3 }
};
volatile uint8_t	SVPWM_sector;
volatile float SVPWM_mag, SVPWM_ang;
volatile float SVPWM_Ti[4];
volatile float SVPWM_Tb1, SVPWM_Tb2;
volatile float SVPWM_beta;

// MOSFET & gate driver variables
int32_t MOSFET_NTC_LookupR[] = {165, 182, 201, 223, 248, 276, 308, 344, 387, 435, 492, 557, 634, 724, 829, 954, 1101, 1277, 1488, 1741, 2048, 2421, 2877, 3438, 4134, 5000, 6087, 7462, 9213, 11462, 14374};
int32_t MOSFET_NTC_LookupT[] = {150, 145, 140, 135, 130, 125, 120, 115, 110, 105, 100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};
uint16_t MOSFET_NTC_LookupSize = 31;
uint8_t driver_RDY = 0; // format: |NA|NA|UH|UL|VH|VL|WH|WL|
uint8_t driver_OK = 0;  // format: |NA|NA|UH|UL|VH|VL|WH|WL|
volatile float duty_u, duty_v, duty_w;
uint8_t U_temp, V_temp, W_temp;
uint32_t NTC_period, NTC_dutyCycle, NTC_Resistance;

// ADC variables
volatile int16_t adc_os[3] = {2000, 2000, 2000};

// timing stuff
uint32_t micros = 0, lastMicros = 0;
float dt_f = 33e-6f; // seconds
volatile int32_t dt_i = 33; // microseconds

// CANBus stuff
volatile bool sendCANBus_flag = false;

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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // Init DWT delay
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;  // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Enable microsecond counter
  LL_TIM_EnableCounter(TIM2);

  // Enable CANBus send interrupt timer
  LL_TIM_EnableCounter(TIM7);
  LL_TIM_EnableIT_UPDATE(TIM7);

  // encoder setup (Emrax motor)
  ENDAT_DIR_Read;
  // LL_SPI_Enable(SPI1);
  LL_SPI_EnableIT_RXNE(SPI1);

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
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.DataLength = FDCAN_DLC_BYTES_6;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.Identifier = CAN_STAT1_ID;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.MessageMarker = 0;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;

  // Enable TIM1, TIM15, and TIM5 (MOSFET temperture PWM signal)
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM15);
  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);

  // Enable HRTIM (gate drive signals)
  writePwm(U_TIMER, 0);
  writePwm(V_TIMER, 0);
  writePwm(W_TIMER, 0);
  LL_HRTIM_EnableOutput(HRTIM1, 
  LL_HRTIM_OUTPUT_TB1|LL_HRTIM_OUTPUT_TB2|
  LL_HRTIM_OUTPUT_TF1|LL_HRTIM_OUTPUT_TF2|
  LL_HRTIM_OUTPUT_TC1|LL_HRTIM_OUTPUT_TC2);
  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_MASTER|U_TIMER|V_TIMER|W_TIMER);

  // Gate driver setup
  resetGateDriver();

  // measure ADC offset
  for (int i = 0; i < 8; i++) {
    LL_mDelay(1);
    adc_os[0] += adc_data[0];
    adc_os[1] += adc_data[1];
    adc_os[2] += adc_data[2];
  }
  adc_os[0] = adc_os[0] >> 3;
  adc_os[1] = adc_os[1] >> 3;
  adc_os[2] = adc_os[2] >> 3;

  // enable HRTIM master interrupt(FOC calculations)
  LL_HRTIM_EnableIT_REP(HRTIM1, LL_HRTIM_TIMER_MASTER);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    micros = TIM2->CNT;
    dt_i = micros - lastMicros;
    dt_f = dt_i * 1e-6f;

    // start read motor position
    motor_lastPhysPosition = motor_PhysPosition;
    #ifdef USE_EMRAX_MOTOR
    //RM44SI encoder
    LL_SPI_TransmitData16(SPI1, 0);
    uint32_t startTime = TIM2->CNT;
    #endif
    #ifdef USE_AMK_MOTOR
    // AMK type-P encoder
    ENDAT_DIR_WRITE;
    //while (LL_SPI_IsActiveFlag_RXNE(SPI1)) LL_SPI_ReceiveData8(SPI1); // clear SPI buffer
    LL_SPI_Disable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_10BIT);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_HALF);
    LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);
    LL_SPI_Enable(SPI1);
    SPI_Wait = true;
    SPI_WaitState = 0;
    SPI_buf = 0;
    motor_lastMeasTime = TIM2->CNT;
    LL_SPI_TransmitData16(SPI1, 0b0000011100U); // send mode 1: Encoder send position values
    #endif

    // convert ADC values to phase current
    U_current = (adc_data[0] - adc_os[0]) * -0.075f;
    V_current = (adc_data[1] - adc_os[1]) * -0.075f;
    W_current = (adc_data[2] - adc_os[2]) * -0.075f;

    // read gate driver status (FLT and RDY pins)
    driver_RDY = (DRV_RDY_UH<<5) | (DRV_RDY_UL<<4) | (DRV_RDY_VH<<3) | (DRV_RDY_VL<<2) | (DRV_RDY_WH<<1) | DRV_RDY_WL;
    driver_OK  = (DRV_FLT_UH<<5) | (DRV_FLT_UL<<4) | (DRV_FLT_VH<<3) | (DRV_FLT_VL<<2) | (DRV_FLT_WH<<1) | DRV_FLT_WL;
    if ((driver_RDY & driver_OK) != 63){ // 0b00111111
      disableGateDriver();
    }

    // calculate motor temperature
    KTY_Temperature = lookupTbl(KTY_LookupR, KTY_LookupT, KTY_LookupSize, adc_data[3] >> 1);

    // measure MOSFETs temperature
    NTC_period    = LL_TIM_OC_GetCompareCH2(TIM1);
    NTC_dutyCycle = LL_TIM_OC_GetCompareCH1(TIM1);
    NTC_Resistance = 24631 * NTC_dutyCycle / NTC_period - 4700;
    U_temp = lookupTbl(MOSFET_NTC_LookupR, MOSFET_NTC_LookupT, MOSFET_NTC_LookupSize, NTC_Resistance);
    NTC_period    = LL_TIM_OC_GetCompareCH1(TIM15);
    NTC_dutyCycle = LL_TIM_OC_GetCompareCH2(TIM15);
    NTC_Resistance = 24631 * (NTC_period - NTC_dutyCycle) / NTC_period - 4700;
    V_temp = lookupTbl(MOSFET_NTC_LookupR, MOSFET_NTC_LookupT, MOSFET_NTC_LookupSize, NTC_Resistance);
    NTC_period    = LL_TIM_OC_GetCompareCH1(TIM5);
    NTC_dutyCycle = LL_TIM_OC_GetCompareCH2(TIM5);
    NTC_Resistance = 24631 * (NTC_period - NTC_dutyCycle) / NTC_period - 4700;
    W_temp = lookupTbl(MOSFET_NTC_LookupR, MOSFET_NTC_LookupT, MOSFET_NTC_LookupSize, NTC_Resistance);

    // wait for encoder communication to finish
    #ifdef USE_EMRAX_MOTOR
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
      if (TIM2->CNT - startTime > 10000U) break;
    }
    motor_PhysPosition = LL_SPI_ReceiveData16(SPI1);
    motor_PhysPosition = (motor_PhysPosition & 0x7FFF) >> 2;
    #endif
    #ifdef USE_AMK_MOTOR
    while (SPI_Wait); // wait for 10-bit command to transfer (reset in interrupt)
    uint32_t uhhh = (SPI_buf >> 10) & (N_STEP_ENCODER-1);
    motor_PhysPosition = uhhh;
    #endif

    // calculate motor speed
    motor_speed = motor_PhysPosition - motor_lastPhysPosition;
    motor_speed = motor_speed * 1000000 / dt_i;

    if (sendCANBus_flag) {
      sendCANBus_flag = false;

      TxHeader.Identifier = CAN_STAT1_ID;
      TxHeader.DataLength = FDCAN_DLC_BYTES_6;
      int16_t AC_current = I_q * 100.0f;
      int16_t DC_current = (I_q + I_d) * SVPWM_mag * 100.0f; // fix formula
      int16_t motor_speed_scaled = motor_PhysPosition;//(motor_speed * 15) >> 14;
      memcpy(&TxData[0], &AC_current, 2);
      memcpy(&TxData[2], &DC_current, 2);
      memcpy(&TxData[4], &motor_speed_scaled, 2);
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);

      TxHeader.Identifier = CAN_STAT2_ID;
      TxHeader.DataLength = FDCAN_DLC_BYTES_3;
      TxData[0] = U_temp + 40;
      TxData[1] = V_temp + 40;
      TxData[2] = W_temp + 40;
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
    }

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
    // LL_mDelay(50);
    while (TIM2->CNT - micros < 33); // AMK encoder sample rate limit
    lastMicros = micros;
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
  duty = 64000 - duty;
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }
    if (RxHeader.Identifier == CAN_CMD_ID) {
      int16_t AC_current_raw;
      memcpy(&AC_current_raw, &RxData[0], 2);
      TargetCurrent = AC_current_raw * 0.01f;
      TargetFieldWk = RxData[6] * 0.1f;
    }
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
