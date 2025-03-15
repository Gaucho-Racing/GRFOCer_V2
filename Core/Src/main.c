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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define PI 3.14159265358979323f
#define PIx2 6.283185307179586f
#define PIo3 1.047197551196598f
#define sqrt3_1o 0.5773502691896258f
#define PI_3o 0.9549296585513721f

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

#define CAN_CMD_ID 0x201708UL
#define CAN_CFG_ID 0x201608UL
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

// Motor variables
int32_t motor_PhysPosition;
float motor_ElecPosition;
float U_current, V_current, W_current;
int32_t KTY_Temperature;
#ifdef USE_EMRAX_MOTOR
uint32_t Encoder_os = 731; // encoder offset angle
int32_t KTY_LookupR[] = {980,1030,1135,1247,1367,1495,1630,1772,1922,2000,2080,2245,2417,2597,2785,2980,3182,3392,3607,3817,3915,4008,4166,4280};
int32_t KTY_LookupT[] = {-55,-50,-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,125,130,140,150};
uint16_t KTY_LookupSize = 24;
#endif
#ifdef USE_AMK_MOTOR
uint32_t Encoder_os = 100;
int32_t KTY_LookupR[] = {359,391,424,460,498,538,581,603,626,672,722,773,826,882,940,1000,1062,1127,1194,1262,1334,1407,1482,1560,1640,1722,1807,1893,1982,2073,2166,2261,2357,2452,2542,2624};
int32_t KTY_LookupT[] = {-40,-30,-20,-10,0,10,20,25,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300};
uint16_t KTY_LookupSize = 36;
#endif

// FOC variables
float sin_elec_position, cos_elec_position;
float I_a, I_b, I_q, I_d;
float cmd_q = 0.0f, cmd_d = 0.0f;
float cmd_a = 0.0f, cmd_b = 0.0f;
float integ_q = 0.0f, integ_d = 0.0f;
float Kp_Iq = 0.01f, Ki_Iq = 0.01f;
float Kp_Id = 0.01f, Ki_Id = 0.01f;
float I_d_err, I_q_err;
volatile float TargetCurrent = 0.0f;
volatile float TargetFieldWk = 0.0f;
const uint8_t SVPWM_PermuataionMatrix[6][3] = {	
	{ 1, 2, 0 },
	{ 3, 1, 0 },
	{ 0, 1, 2 },
	{ 0, 3, 1 },
	{ 2, 0, 1 },
	{ 1, 0, 3 }
};
uint8_t	SVPWM_sector;
float SVPWM_mag, SVPWM_ang;
float SVPWM_Ti[4];
float SVPWM_Tb1, SVPWM_Tb2;
float SVPWM_beta;

// MOSFET & gate driver variables
int32_t MOSFET_NTC_LookupR[] = {165, 182, 201, 223, 248, 276, 308, 344, 387, 435, 492, 557, 634, 724, 829, 954, 1101, 1277, 1488, 1741, 2048, 2421, 2877, 3438, 4134, 5000, 6087, 7462, 9213, 11462, 14374};
int32_t MOSFET_NTC_LookupT[] = {150, 145, 140, 135, 130, 125, 120, 115, 110, 105, 100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};
uint16_t MOSFET_NTC_LookupSize = 31;
uint8_t driver_RDY = 0; // format: |NA|NA|UH|UL|VH|VL|WH|WL|
uint8_t driver_OK = 0;  // format: |NA|NA|UH|UL|VH|VL|WH|WL|
int32_t duty_u, duty_v, duty_w;
uint8_t U_temp, V_temp, W_temp;
uint32_t NTC_period, NTC_dutyCycle, NTC_Resistance;

// ADC variables
int16_t adc_os[3] = {0, 0, 0};

// timing stuff
uint32_t micros = 0, lastMicros = 0;
float dt = 1e-4f;
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
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  while (1)
  {
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
    //while (LL_SPI_IsActiveFlag_RXNE(SPI1)) LL_SPI_ReceiveData8(SPI1); // clear SPI buffer
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
    // read gate driver status (FLT and RDY pins)
    driver_RDY = (DRV_RDY_UH<<5) | (DRV_RDY_UL<<4) | (DRV_RDY_VH<<3) | (DRV_RDY_VL<<2) | (DRV_RDY_WH<<1) | DRV_RDY_WL;
    driver_OK  = (DRV_FLT_UH<<5) | (DRV_FLT_UL<<4) | (DRV_FLT_VH<<3) | (DRV_FLT_VL<<2) | (DRV_FLT_WH<<1) | DRV_FLT_WL;
    if ((driver_RDY & driver_OK) != 63){ // 0b00111111
      disableGateDriver();
    }

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
    LL_SPI_TransmitData16(SPI1, 0U);
    LL_SPI_TransmitData16(SPI1, 0U);

    // again, do math while waiting for SPI
    // Clarke transform
    I_a = U_current * 0.66666667f - V_current * 0.33333333f - W_current * 0.33333333f;
    I_b = 0.5773502691896257f * (V_current - W_current);
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

    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    buf |= LL_SPI_ReceiveData16(SPI1) >> 8;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    buf |= ((uint32_t)LL_SPI_ReceiveData16(SPI1)) << 8;
    #endif


    motor_PhysPosition = N_STEP_ENCODER-1 - (buf & (N_STEP_ENCODER - 1));
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);
    //motor_ElecPosition = fmodf((float)(motor_PhysPosition + Encoder_os) / N_STEP_ENCODER * N_POLES, 1.0f) * PI * 2.0f; // radians
    motor_ElecPosition = (float)((motor_PhysPosition + Encoder_os) % (N_STEP_ENCODER / N_POLES)) / (N_STEP_ENCODER / N_POLES) * PIx2;
    // FOC
    arm_sin_cos_f32(motor_ElecPosition, &sin_elec_position, &cos_elec_position);
    // sin_elec_position = sinf(motor_ElecPosition);
    // cos_elec_position = cosf(motor_ElecPosition);
    // Park transform
    I_d = I_a * cos_elec_position + I_b * sin_elec_position;
    I_q = I_b * cos_elec_position - I_a * sin_elec_position;
    // PI controllers on Q and D
    I_d_err = TargetFieldWk - I_d;
    I_q_err = TargetCurrent - I_q;
    integ_d += I_d_err * Ki_Id;
    integ_d = (integ_d > 0.5f)  ?  0.5f : integ_d;
    integ_d = (integ_d < -0.5f) ? -0.5f : integ_d;
    cmd_d = I_d_err * Kp_Id + integ_d;
    integ_q += I_q_err * Ki_Iq;
    integ_q = (integ_q > 0.57f)  ?  0.57f : integ_q;
    integ_q = (integ_q < -0.57f) ? -0.57f : integ_q;
    cmd_q = I_q_err * Kp_Iq + integ_q;
    // Inverse Park transform
    cmd_a = cmd_d * cos_elec_position - cmd_q * sin_elec_position;
    cmd_b = cmd_q * cos_elec_position + cmd_d * sin_elec_position;
    // Inverse Clarke transform
    arm_sqrt_f32(cmd_a*cmd_a + cmd_b*cmd_b, &SVPWM_mag);
    arm_atan2_f32(cmd_b, cmd_a, &SVPWM_ang);
    SVPWM_ang += PI;
    SVPWM_mag = (SVPWM_mag > sqrt3_1o) ? sqrt3_1o : SVPWM_mag;
    SVPWM_sector = (uint8_t)(SVPWM_ang * PI_3o);
    SVPWM_beta = SVPWM_ang - PIo3 * SVPWM_sector;
    SVPWM_Tb1 = SVPWM_mag * arm_sin_f32(PIo3 - SVPWM_beta);
    SVPWM_Tb2 = SVPWM_mag * arm_sin_f32(SVPWM_beta);
    SVPWM_Ti[0] = (1.0f - SVPWM_Tb1 - SVPWM_Tb2) * 0.5f;
    SVPWM_Ti[1] = SVPWM_Tb1 + SVPWM_Tb2 + SVPWM_Ti[0];
    SVPWM_Ti[2] = SVPWM_Tb2 + SVPWM_Ti[0];
    SVPWM_Ti[3] = SVPWM_Tb1 + SVPWM_Ti[0];
    // Update duty cycle
    duty_u = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][0]] * 64000.0f;
    duty_v = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][1]] * 64000.0f;
    duty_w = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][2]] * 64000.0f;
    writePwm(U_TIMER, duty_u);
    writePwm(V_TIMER, duty_v);
    writePwm(W_TIMER, duty_w);
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
    

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
      int16_t AC_current_raw = (int16_t)RxData[0] << 8;
      AC_current_raw += RxData[1];
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
