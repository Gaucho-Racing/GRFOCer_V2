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
#include "FastMath.h"
#include "FOC.h"
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
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[64];

char printBuffer[1024];

// FOC variables
volatile FOC_data* FOC;

// Motor variables
volatile bool SPI_Wait = false;
volatile uint8_t SPI_WaitState = 0;
volatile uint32_t SPI_buf = 0U;
int32_t motor_PhysPosition;
int32_t motor_lastPhysPosition;
volatile uint32_t motor_lastMeasTime, motor_lastMeasTime2;
uint32_t motor_last2MeasTime;
volatile float motor_speed = 0.0f; // encoder LSBs / us
int32_t KTY_Temperature;
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

// MOSFET & gate driver variables
int32_t MOSFET_NTC_LookupR[] = {165, 182, 201, 223, 248, 276, 308, 344, 387, 435, 492, 557, 634, 724, 829, 954, 1101, 1277, 1488, 1741, 2048, 2421, 2877, 3438, 4134, 5000, 6087, 7462, 9213, 11462, 14374};
int32_t MOSFET_NTC_LookupT[] = {150, 145, 140, 135, 130, 125, 120, 115, 110, 105, 100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};
uint16_t MOSFET_NTC_LookupSize = 31;
uint8_t driver_RDY = 0; // format: |NA|NA|UH|UL|VH|VL|WH|WL|
uint8_t driver_OK = 0;  // format: |NA|NA|UH|UL|VH|VL|WH|WL|
uint8_t U_temp, V_temp, W_temp;
uint32_t NTC_period, NTC_dutyCycle, NTC_Resistance;

// ADC variables
volatile int16_t adc_data[64];
volatile int16_t adc_os[3] = {0, 0, 0};

// timing stuff
uint32_t micros = 0, lastMicros = 0;
float dt_f = 33e-6f; // seconds
volatile int32_t dt_i = 50; // microseconds

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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  disableGateDriver();

  // allocate memory on heap for FOC data
  FOC = (FOC_data*)malloc(sizeof(FOC_data));
  if (FOC == NULL) {
    // Handle memory allocation failure
    disableGateDriver();
    while (1);
  }

  // Disable FPU lazy context save
  FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;

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

  // Enable TIM6 (Drive-enable timeout interrupt)
  LL_TIM_EnableCounter(TIM6);
  LL_TIM_EnableIT_UPDATE(TIM6);

  // initialize FOC variables
  FOC->Encoder_os = Encoder_os;
  FOC->Kp_Id = 0.0f; FOC->Ki_Id = 10.0f;
  FOC->Kp_Iq = 0.0f; FOC->Ki_Iq = 10.0f;
  FOC->integ_d = 0.0f;
  FOC->integ_q = 0.0f;
  FOC->motor_speed = 0.0f;
  FOC->TargetCurrent = 0.0f;
  FOC->TargetFieldWk = 0.0f;

  // measure ADC offset
  for (uint16_t i = 0; i < 100; i++){
    LL_mDelay(10);
    adc_os[0] = (adc_os[0]*3 + adc_data[0]) >> 2;
    adc_os[1] = (adc_os[1]*3 + adc_data[1]) >> 2;
    adc_os[2] = (adc_os[2]*3 + adc_data[2]) >> 2;
  }

  // Enable HRTIM (gate drive signals)
  FOC->F_sw = LL_HRTIM_TIM_GetPrescaler(HRTIM1, LL_HRTIM_TIMER_B);
  writePwm(U_TIMER, 0);
  writePwm(V_TIMER, 0);
  writePwm(W_TIMER, 0);
  LL_HRTIM_EnableOutput(HRTIM1, 
  LL_HRTIM_OUTPUT_TB1|LL_HRTIM_OUTPUT_TB2|
  LL_HRTIM_OUTPUT_TF1|LL_HRTIM_OUTPUT_TF2|
  LL_HRTIM_OUTPUT_TC1|LL_HRTIM_OUTPUT_TC2);
  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_MASTER|U_TIMER|V_TIMER|W_TIMER);

  // enable HRTIM timer B interrupt(FOC calculations)
  LL_HRTIM_EnableIT_UPDATE(HRTIM1, LL_HRTIM_TIMER_B);
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
    // __disable_irq();
    motor_PhysPosition = (SPI_buf >> 4) & (N_STEP_ENCODER-1);
    motor_lastMeasTime = motor_lastMeasTime2;
    // __enable_irq();
    #endif

    // calculate motor speed
    int32_t motor_PhysPositionDiff = motor_PhysPosition - motor_lastPhysPosition;
    if (motor_PhysPositionDiff > (N_STEP_ENCODER >> 1)) {
      motor_PhysPositionDiff -= N_STEP_ENCODER;
    } else if (motor_PhysPositionDiff < -(N_STEP_ENCODER >> 1)) {
      motor_PhysPositionDiff += N_STEP_ENCODER;
    }
    float motor_speed_new = (float)motor_PhysPositionDiff / (float)(motor_lastMeasTime - motor_last2MeasTime);
    motor_speed += (motor_speed_new - motor_speed) * 0.03f;

    // move encoder info to FOC struct
    __disable_irq();
    FOC->motor_PhysPosition = motor_PhysPosition;
    FOC->motor_lastMeasTime = motor_lastMeasTime;
    FOC->motor_speed = motor_speed;
    FOC->U_current = (adc_data[0] - adc_os[0]) * 0.075f;
    FOC->V_current = (adc_data[1] - adc_os[1]) * 0.075f;
    FOC->W_current = (adc_data[2] - adc_os[2]) * 0.075f;
    // Flush pipeline
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __enable_irq();

    // FOC_update(FOC);

    if (sendCANBus_flag != 0){
      if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) != 0){
        switch (sendCANBus_flag) {
          case 2:
            TxHeader.Identifier = CAN_STAT1_ID;
            TxHeader.DataLength = FDCAN_DLC_BYTES_6;
            int16_t AC_current = FOC->I_q_avg * 100.0f;
            int16_t DC_current = FOC->I_d_avg * 100.0f; // TODO: find right formula
            int16_t motor_speed_scaled = FOC->motor_speed * 60e6f / N_STEP_ENCODER;
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
            int16_t temp = FOC->cmd_q * 100.0f;
            memcpy(&TxData[0], &temp, 2);
            temp = FOC->cmd_d * 100.0f;
            memcpy(&TxData[2], &temp, 2);
            temp = FOC->motor_ElecPosition * 10000.0f;
            memcpy(&TxData[4], &temp, 2);
            uint16_t temp2 = FOC->motor_PhysPosition;
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
    while (TIM2->CNT - micros < 50); // AMK encoder sample rate limit
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
  FOC->TargetCurrent = 0.0f;
  FOC->TargetFieldWk = 0.0f;
  FOC->integ_d = 0.0f;
  FOC->integ_q = 0.0f;
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
      if (RxData[7] != 1) {
        disableGateDriver();
      }
      else {
        LL_TIM_SetCounter(TIM6, 0); // reset drive-enable timeout counter
        if ((LL_GPIO_ReadInputPort(GPIOC) & LL_GPIO_PIN_12) == 0) {
          resetGateDriver();
        }
      }
      FOC->TargetCurrent = AC_current_raw * 0.01f;
      FOC->TargetFieldWk = RxData[6] * -0.1f;
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
