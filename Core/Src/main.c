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
// #include "arm_math.h"
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
volatile int16_t adc_data[64];

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[64];

char printBuffer[1024];

// Fast math tables
const float sin_LookupX[] = {
  0.  , 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1 ,
  0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2 , 0.21,
  0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 0.3 , 0.31, 0.32,
  0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.4 , 0.41, 0.42, 0.43,
  0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.5 , 0.51, 0.52, 0.53, 0.54,
  0.55, 0.56, 0.57, 0.58, 0.59, 0.6 , 0.61, 0.62, 0.63, 0.64, 0.65,
  0.66, 0.67, 0.68, 0.69, 0.7 , 0.71, 0.72, 0.73, 0.74, 0.75, 0.76,
  0.77, 0.78, 0.79, 0.8 , 0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87,
  0.88, 0.89, 0.9 , 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98,
  0.99};
const int32_t sin_LookupXl[] = {
      0,   661,  1323,  1985,  2647,  3309,  3971,  4633,  5295,
   5957,  6619,  7281,  7943,  8605,  9267,  9929, 10591, 11253,
  11915, 12577, 13239, 13901, 14563, 15225, 15887, 16549, 17211,
  17873, 18535, 19197, 19859, 20521, 21183, 21845, 22506, 23168,
  23830, 24492, 25154, 25816, 26478, 27140, 27802, 28464, 29126,
  29788, 30450, 31112, 31774, 32436, 33098, 33760, 34422, 35084,
  35746, 36408, 37070, 37732, 38394, 39056, 39718, 40380, 41042,
  41704, 42366, 43028, 43690, 44351, 45013, 45675, 46337, 46999,
  47661, 48323, 48985, 49647, 50309, 50971, 51633, 52295, 52957,
  53619, 54281, 54943, 55605, 56267, 56929, 57591, 58253, 58915,
  59577, 60239, 60901, 61563, 62225, 62887, 63549, 64211, 64873,
  65535
};
const float sin_LookupY[] = {
  0.00000000e+00,  6.27905195e-02,  1.25333234e-01,  1.87381315e-01,
  2.48689887e-01,  3.09016994e-01,  3.68124553e-01,  4.25779292e-01,
  4.81753674e-01,  5.35826795e-01,  5.87785252e-01,  6.37423990e-01,
  6.84547106e-01,  7.28968627e-01,  7.70513243e-01,  8.09016994e-01,
  8.44327926e-01,  8.76306680e-01,  9.04827052e-01,  9.29776486e-01,
  9.51056516e-01,  9.68583161e-01,  9.82287251e-01,  9.92114701e-01,
  9.98026728e-01,  1.00000000e+00,  9.98026728e-01,  9.92114701e-01,
  9.82287251e-01,  9.68583161e-01,  9.51056516e-01,  9.29776486e-01,
  9.04827052e-01,  8.76306680e-01,  8.44327926e-01,  8.09016994e-01,
  7.70513243e-01,  7.28968627e-01,  6.84547106e-01,  6.37423990e-01,
  5.87785252e-01,  5.35826795e-01,  4.81753674e-01,  4.25779292e-01,
  3.68124553e-01,  3.09016994e-01,  2.48689887e-01,  1.87381315e-01,
  1.25333234e-01,  6.27905195e-02,  1.22464680e-16, -6.27905195e-02,
  -1.25333234e-01, -1.87381315e-01, -2.48689887e-01, -3.09016994e-01,
  -3.68124553e-01, -4.25779292e-01, -4.81753674e-01, -5.35826795e-01,
  -5.87785252e-01, -6.37423990e-01, -6.84547106e-01, -7.28968627e-01,
  -7.70513243e-01, -8.09016994e-01, -8.44327926e-01, -8.76306680e-01,
  -9.04827052e-01, -9.29776486e-01, -9.51056516e-01, -9.68583161e-01,
  -9.82287251e-01, -9.92114701e-01, -9.98026728e-01, -1.00000000e+00,
  -9.98026728e-01, -9.92114701e-01, -9.82287251e-01, -9.68583161e-01,
  -9.51056516e-01, -9.29776486e-01, -9.04827052e-01, -8.76306680e-01,
  -8.44327926e-01, -8.09016994e-01, -7.70513243e-01, -7.28968627e-01,
  -6.84547106e-01, -6.37423990e-01, -5.87785252e-01, -5.35826795e-01,
  -4.81753674e-01, -4.25779292e-01, -3.68124553e-01, -3.09016994e-01,
  -2.48689887e-01, -1.87381315e-01, -1.25333234e-01, -6.27905195e-02
};
const float cos_LookupY[] = {
  1.00000000e+00,  9.98026728e-01,  9.92114701e-01,  9.82287251e-01,
  9.68583161e-01,  9.51056516e-01,  9.29776486e-01,  9.04827052e-01,
  8.76306680e-01,  8.44327926e-01,  8.09016994e-01,  7.70513243e-01,
  7.28968627e-01,  6.84547106e-01,  6.37423990e-01,  5.87785252e-01,
  5.35826795e-01,  4.81753674e-01,  4.25779292e-01,  3.68124553e-01,
  3.09016994e-01,  2.48689887e-01,  1.87381315e-01,  1.25333234e-01,
  6.27905195e-02,  6.12323400e-17, -6.27905195e-02, -1.25333234e-01,
  -1.87381315e-01, -2.48689887e-01, -3.09016994e-01, -3.68124553e-01,
  -4.25779292e-01, -4.81753674e-01, -5.35826795e-01, -5.87785252e-01,
  -6.37423990e-01, -6.84547106e-01, -7.28968627e-01, -7.70513243e-01,
  -8.09016994e-01, -8.44327926e-01, -8.76306680e-01, -9.04827052e-01,
  -9.29776486e-01, -9.51056516e-01, -9.68583161e-01, -9.82287251e-01,
  -9.92114701e-01, -9.98026728e-01, -1.00000000e+00, -9.98026728e-01,
  -9.92114701e-01, -9.82287251e-01, -9.68583161e-01, -9.51056516e-01,
  -9.29776486e-01, -9.04827052e-01, -8.76306680e-01, -8.44327926e-01,
  -8.09016994e-01, -7.70513243e-01, -7.28968627e-01, -6.84547106e-01,
  -6.37423990e-01, -5.87785252e-01, -5.35826795e-01, -4.81753674e-01,
  -4.25779292e-01, -3.68124553e-01, -3.09016994e-01, -2.48689887e-01,
  -1.87381315e-01, -1.25333234e-01, -6.27905195e-02, -1.83697020e-16,
  6.27905195e-02,  1.25333234e-01,  1.87381315e-01,  2.48689887e-01,
  3.09016994e-01,  3.68124553e-01,  4.25779292e-01,  4.81753674e-01,
  5.35826795e-01,  5.87785252e-01,  6.37423990e-01,  6.84547106e-01,
  7.28968627e-01,  7.70513243e-01,  8.09016994e-01,  8.44327926e-01,
  8.76306680e-01,  9.04827052e-01,  9.29776486e-01,  9.51056516e-01,
  9.68583161e-01,  9.82287251e-01,  9.92114701e-01,  9.98026728e-01
};
const uint16_t math_LookupSize = 100;

// Motor variables
volatile int32_t temp_it, temp_itPrev, temp_itPrev2;
volatile bool SPI_Wait = false;
volatile uint8_t SPI_WaitState = 0;
volatile uint32_t SPI_buf = 0U;
volatile int32_t motor_PhysPosition;
int32_t motor_lastPhysPosition;
volatile uint32_t motor_lastMeasTime, motor_lastMeasTime2;
uint32_t motor_last2MeasTime;
volatile float motor_ElecPosition;
volatile float U_current, V_current, W_current;
volatile float motor_speed = 0.0f; // encoder LSBs / us
volatile float RpmSafetyMult;
int32_t KTY_Temperature;
volatile uint8_t glitchCount = 0;
#ifdef USE_EMRAX_MOTOR
volatile int32_t Encoder_os = 449; // encoder offset angle
const int32_t KTY_LookupR[] = {980,1030,1135,1247,1367,1495,1630,1772,1922,2000,2080,2245,2417,2597,2785,2980,3182,3392,3607,3817,3915,4008,4166,4280};
const int32_t KTY_LookupT[] = {-55,-50,-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,125,130,140,150};
const uint16_t KTY_LookupSize = 24;
#endif
#ifdef USE_AMK_MOTOR
static const int32_t Encoder_os = 4450;
const int32_t KTY_LookupR[] = {359,391,424,460,498,538,581,603,626,672,722,773,826,882,940,1000,1062,1127,1194,1262,1334,1407,1482,1560,1640,1722,1807,1893,1982,2073,2166,2261,2357,2452,2542,2624};
const int32_t KTY_LookupT[] = {-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300};
const uint16_t KTY_LookupSize = 36;
#endif

// FOC variables
volatile uint32_t F_sw;
volatile float sin_elec_position, cos_elec_position;
volatile float I_a, I_b, I_q, I_d;
volatile float I_q_avg, I_d_avg;
volatile float cmd_q = 0.0f, cmd_d = 0.0f;
volatile float cmd_a = 0.0f, cmd_b = 0.0f;
volatile float integ_q = 0.0f, integ_d = 0.0f;
volatile float Kp_Iq = 0.0f, Ki_Iq = 1.0f;
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
volatile int16_t adc_os[3] = {0, 0, 0};

// timing stuff
uint32_t micros = 0, lastMicros = 0;
float dt_f = 33e-6f; // seconds
volatile int32_t dt_i = 33; // microseconds

// CANBus stuff
volatile uint8_t sendCANBus_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void printCANBus(char* text);

void resetGateDriver();
void disableGateDriver();
void writePwm(uint32_t timer, int32_t duty);

void FOC();

int32_t lookupTbl(const int32_t* source, const int32_t* target, const uint32_t size, const int32_t value);
float lookupTblf(const float* source, const float* target, const uint32_t size, const float value);
float flookupTbll(const int32_t* source, const float* target, const uint32_t size, const int32_t value);
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

  // encoder setup
  ENDAT_DIR_Read;
  #ifdef USE_EMRAX_MOTOR
  LL_SPI_Enable(SPI1);
  #endif
  #ifdef USE_AMK_MOTOR
  LL_SPI_EnableIT_RXNE(SPI1);
  #endif

  // Enable ADC1 (phase current sensors) with DMA
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&adc_data[0]);
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
  F_sw = LL_HRTIM_TIM_GetPrescaler(HRTIM1, LL_HRTIM_TIMER_B);
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

  // enable HRTIM timer B interrupt(FOC calculations)
  LL_HRTIM_EnableIT_UPDATE(HRTIM1, LL_HRTIM_TIMER_B);

  // measure ADC offset
  for (uint16_t i = 0; i < 100; i++){
    LL_mDelay(10);
    adc_os[0] = (adc_os[0]*3 + adc_data[0]) >> 2;
    adc_os[1] = (adc_os[1]*3 + adc_data[1]) >> 2;
    adc_os[2] = (adc_os[2]*3 + adc_data[2]) >> 2;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool isFirstCycle = true;
  while (1)
  {
    micros = TIM2->CNT;
    dt_i = micros - lastMicros;
    dt_f = dt_i * 1e-6f;

    // start read motor position
    motor_lastPhysPosition = motor_PhysPosition;
    motor_last2MeasTime = motor_lastMeasTime;
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
    LL_SPI_TransmitData16(SPI1, 28U); // send mode 1 (0b0000011100): Encoder send position values
    #endif

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
    motor_lastMeasTime = TIM2->CNT - 7;
    motor_PhysPosition = LL_SPI_ReceiveData16(SPI1);
    motor_PhysPosition = (motor_PhysPosition & 0x7FFF) >> 2;
    #endif
    #ifdef USE_AMK_MOTOR
    while (SPI_Wait); // wait for 10-bit command to transfer (reset in interrupt)
    uint8_t shiftAmount = 0;
    while ((SPI_buf & 1UL) == 0){
      SPI_buf = SPI_buf >> 1;
      shiftAmount++;
    }
    __disable_irq();
    motor_PhysPosition = (SPI_buf >> 4) & (N_STEP_ENCODER-1);
    motor_lastMeasTime = motor_lastMeasTime2;
    __enable_irq();
    #endif

    // calculate motor speed
    int32_t motor_PhysPositionDiff = motor_PhysPosition - motor_lastPhysPosition;
    if (motor_PhysPositionDiff > (N_STEP_ENCODER >> 1)) {
      motor_PhysPositionDiff -= N_STEP_ENCODER;
    } else if (motor_PhysPositionDiff < -(N_STEP_ENCODER >> 1)) {
      motor_PhysPositionDiff += N_STEP_ENCODER;
    }
    float motor_speed_new = (float)motor_PhysPositionDiff / (float)(motor_lastMeasTime - motor_last2MeasTime);
    __disable_irq();
    motor_speed += (motor_speed_new - motor_speed) * 0.03f;
    __enable_irq();
    if (isFirstCycle) {
      isFirstCycle = false;
      temp_it = motor_PhysPosition + Encoder_os;
      temp_it *= N_POLES;
      temp_it %= N_STEP_ENCODER;
      temp_itPrev = temp_it;
      temp_itPrev2 = temp_it;
    }

    if (sendCANBus_flag != 0){
      if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) != 0){
        switch (sendCANBus_flag) {
          case 2:
            TxHeader.Identifier = CAN_STAT1_ID;
            TxHeader.DataLength = FDCAN_DLC_BYTES_6;
            int16_t AC_current = I_q * 100.0f;
            int16_t DC_current = I_d * 100.0f; // TODO: find right formula
            int16_t motor_speed_scaled = motor_speed * 60e6f / N_STEP_ENCODER;
            memcpy(&TxData[0], &AC_current, 2);
            memcpy(&TxData[2], &DC_current, 2);
            memcpy(&TxData[4], &motor_speed_scaled, 2);
            sendCANBus_flag--;
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
            break;
          case 3:
            TxHeader.Identifier = CAN_STAT2_ID;
            TxHeader.DataLength = FDCAN_DLC_BYTES_3;
            TxData[0] = U_temp + 40;
            TxData[1] = V_temp + 40;
            TxData[2] = W_temp + 40;
            sendCANBus_flag--;
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
            break;
          case 4:
            TxHeader.Identifier = CAN_STAT3_ID;
            TxHeader.DataLength = FDCAN_DLC_BYTES_2;
            TxData[0] = KTY_Temperature + 40;
            TxData[1] = 0; // TODO
            sendCANBus_flag--;
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
            break;
          case 1:
            TxHeader.Identifier = CAN_DEBUG_ID;
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            int16_t temp = Encoder_os;
            memcpy(&TxData[0], &temp, 2);
            temp = temp_it;
            memcpy(&TxData[2], &temp, 2);
            temp = motor_ElecPosition * 10000.0f;
            memcpy(&TxData[4], &temp, 2);
            uint16_t temp2 = motor_PhysPosition;
            memcpy(&TxData[6], &temp2, 2);
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
            sendCANBus_flag--;
            break;
          default:
            break;
        }
      }
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
  // duty = (duty > 62000) ? 62000 : duty;
  // duty = (duty < 2000) ? 2000 : duty;
  uint32_t duty_H, duty_L;
  if (duty <= deadTime) {
    duty_H = 0;
    duty_L = 0;
  }
  else if (duty >= 64000 - deadTime) {
    duty_H = 64000;
    duty_L = 64000;
  }
  else {
    if (duty <= deadTime << 1) {
      duty_H = 0;
      duty_L = duty + deadTime;
    }
    else if (duty >= 64000 - (deadTime << 1)) {
      duty_H = duty - deadTime;
      duty_L = 64000;
    }
    else {
      duty_H = duty - deadTime;
      duty_L = duty + deadTime;
    }
  }
  duty_H = (duty_H > 63900) ? 63900 : duty_H;
  duty_L = (duty_L > 63900) ? 63900 : duty_L;
  LL_HRTIM_TIM_SetCompare1(HRTIM1, timer, duty_H);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, timer, duty_L);
}

void FOC() {
  // convert ADC values to phase current
  U_current = (adc_data[0] - adc_os[0]) * 0.075f;
  V_current = (adc_data[1] - adc_os[1]) * 0.075f;
  W_current = (adc_data[2] - adc_os[2]) * 0.075f;
  // FOC
  // motor_ElecPosition = fmodf(((float)(motor_PhysPosition + Encoder_os) + (TIM2->CNT - motor_lastMeasTime + (13 << F_sw))*motor_speed)
  //   * (float)N_POLES / (float)N_STEP_ENCODER, 1.0f);
  temp_it = motor_PhysPosition + Encoder_os;
  if (fabsf(motor_speed) > ((300.0f/60.0f)*1e-6f*N_STEP_ENCODER * 0.02f)){
    temp_it += (TIM2->CNT - motor_lastMeasTime + (13 << F_sw))*motor_speed;
  }
  temp_it *= N_POLES;
  temp_it &= 0xFFFF;
  uint16_t uhhh = temp_it;
  // if (abs(((temp_it - temp_itPrev)&0xFFFF) - ((temp_itPrev - temp_itPrev2)&0xFFFF)) > 10000){
  //   glitchCount++;
  //   temp_it = (temp_itPrev + (temp_itPrev - temp_itPrev2)) & 0xFFFF;
  // }
  // else {
  //   glitchCount = 0;
  // }
  // if (glitchCount > 1) {
  //   glitchCount = 0;
  //   temp_it = uhhh;
  // }
  temp_itPrev2 = temp_itPrev;
  temp_itPrev = temp_it;
  // uint16_t temp3 = temp_it;
  // memcpy(&TxDataIT[2], &temp3, 2);
  motor_ElecPosition = (float)temp_it / (float)N_STEP_ENCODER;
  // motor_ElecPosition = 0.0f;
  // sin_elec_position = lookupTblf(sin_LookupX, sin_LookupY, math_LookupSize, motor_ElecPosition);
  // cos_elec_position = lookupTblf(sin_LookupX, cos_LookupY, math_LookupSize, motor_ElecPosition);
  sin_elec_position = flookupTbll(sin_LookupXl, sin_LookupY, math_LookupSize, uhhh);
  cos_elec_position = flookupTbll(sin_LookupXl, cos_LookupY, math_LookupSize, uhhh);
  // sin_elec_position = sinf(motor_ElecPosition * PIx2);
  // cos_elec_position = cosf(motor_ElecPosition * PIx2);
  // Clarke transform
  I_a = U_current * 0.66666667f - V_current * 0.33333333f - W_current * 0.33333333f;
  I_b = sqrt3_1o * (V_current - W_current);
  // Park transform
  I_d = I_a * cos_elec_position + I_b * sin_elec_position;
  I_q = I_b * cos_elec_position - I_a * sin_elec_position;
  I_d_avg += (I_d - I_d_avg) * 0.001f;
  I_q_avg += (I_q - I_q_avg) * 0.001f;
  // PI controllers on Q and D
  RpmSafetyMult = (fabsf(motor_speed) > MAX_SPEED * 0.9) ? 
    (MAX_SPEED - fabsf(motor_speed)) / MAX_SPEED * 10.0f : 1.0f;
  I_d_err = TargetFieldWk * RpmSafetyMult - I_d;
  I_q_err = TargetCurrent * RpmSafetyMult - I_q;
  integ_d += I_d_err * Ki_Id * 25e-6f * ((float)(1U << F_sw));
  integ_d = (integ_d > MAX_CMD_D) ? MAX_CMD_D : integ_d;
  integ_d = (integ_d < MIN_CMD_D) ? MIN_CMD_D : integ_d;
  cmd_d = I_d_err * Kp_Id + integ_d;
  integ_q += I_q_err * Ki_Iq * 25e-6f * ((float)(1U << F_sw));
  integ_q = (integ_q > MAX_CMD_Q) ? MAX_CMD_Q : integ_q;
  integ_q = (integ_q < MIN_CMD_Q) ? MIN_CMD_Q : integ_q;
  cmd_q = I_q_err * Kp_Iq + integ_q;
  // Inverse Park transform
  cmd_a = cmd_d * cos_elec_position - cmd_q * sin_elec_position;
  cmd_b = cmd_q * cos_elec_position + cmd_d * sin_elec_position;
  // Inverse Clarke transform
  duty_u = cmd_a;
  duty_v = (cmd_a * -0.5f + 0.8660254037844386f * cmd_b);
  duty_w = (cmd_a * -0.5f - 0.8660254037844386f * cmd_b);
  writePwm(U_TIMER, duty_u * -32000.0f + 32000);
  writePwm(V_TIMER, duty_v * -32000.0f + 32000);
  writePwm(W_TIMER, duty_w * -32000.0f + 32000);
  // SVPWM generation
  // SVPWM_mag = hypotf(cmd_b, cmd_a);
  // SVPWM_ang = atan2f(cmd_b, cmd_a);
  // SVPWM_ang += PI;
  // SVPWM_sector = (uint8_t)(SVPWM_ang * PI_3o);
  // SVPWM_beta = SVPWM_ang - PIo3 * SVPWM_sector;
  // SVPWM_Tb1 = SVPWM_mag * sinf(PIo3 - SVPWM_beta);
  // SVPWM_Tb2 = SVPWM_mag * sinf(SVPWM_beta);
  // SVPWM_Ti[0] = (1.0f - SVPWM_Tb1 - SVPWM_Tb2) * 0.5f;
  // SVPWM_Ti[1] = SVPWM_Tb1 + SVPWM_Tb2 + SVPWM_Ti[0];
  // SVPWM_Ti[2] = SVPWM_Tb2 + SVPWM_Ti[0];
  // SVPWM_Ti[3] = SVPWM_Tb1 + SVPWM_Ti[0];
  // duty_u = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][0]];
  // duty_v = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][1]];
  // duty_w = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][2]];
  // writePwm(U_TIMER, duty_u * 64000.0f);
  // writePwm(V_TIMER, duty_v * 64000.0f);
  // writePwm(W_TIMER, duty_w * 64000.0f);
  // if (sendCANBus_flag == 0) sendCANBus_flag = 1;
  
  // TxHeaderIT.BitRateSwitch = FDCAN_BRS_OFF;
  // TxHeaderIT.DataLength = FDCAN_DLC_BYTES_8;
  // TxHeaderIT.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  // TxHeaderIT.FDFormat = FDCAN_CLASSIC_CAN;
  // TxHeaderIT.Identifier = CAN_DEBUG_ID;
  // TxHeaderIT.IdType = FDCAN_EXTENDED_ID;
  // TxHeaderIT.MessageMarker = 0;
  // TxHeaderIT.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  // TxHeaderIT.TxFrameType = FDCAN_DATA_FRAME;
  // uint32_t testtest = 0x0UL;
  // TxDataIT[0] = (uint8_t)0x00;
  // TxDataIT[1] = (uint8_t)0x00;
  // TxDataIT[2] = (uint8_t)0x00;
  // TxDataIT[3] = (uint8_t)0x00;
  // // memcpy(&TxDataIT[0], &testtest, 4);
  // uint16_t temp = motor_ElecPosition * 10000.0f;
  // memcpy(&TxDataIT[4], &temp, 2);
  // uint16_t temp2 = motor_PhysPosition;
  // memcpy(&TxDataIT[6], &temp2, 2);
  // HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeaderIT, TxDataIT);
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

float lookupTblf(const float* source, const float* target, const uint32_t size, const float value){
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

float flookupTbll(const int32_t* source, const float* target, const uint32_t size, const int32_t value){
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
      TargetFieldWk = RxData[6] * -0.1f;
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
