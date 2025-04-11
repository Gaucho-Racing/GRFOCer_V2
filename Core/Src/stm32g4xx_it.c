/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "defines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t x = 0;

FDCAN_TxHeaderTypeDef TxHeaderIT;
uint8_t TxDataIT[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan2;
/* USER CODE BEGIN EV */
// extern volatile FDCAN_RxHeaderTypeDef RxHeader;
// extern volatile uint8_t RxData[8];
// extern volatile FDCAN_TxHeaderTypeDef TxHeader;
// extern volatile uint8_t TxData[8];

extern volatile int32_t dt_i;
extern volatile int32_t temp_it, temp_itPrev, temp_itPrev2;
extern volatile int16_t adc_data[4];
extern volatile int16_t adc_os[3];

extern volatile bool SPI_Wait;
extern volatile uint8_t SPI_WaitState;
extern volatile uint32_t SPI_buf;

extern volatile float motor_ElecPosition;
extern volatile int32_t motor_PhysPosition;
extern volatile int32_t Encoder_os;
extern volatile uint32_t motor_lastMeasTime, motor_lastMeasTime2; // 2 is for saving data temporarity before moving it to 1
extern volatile float motor_speed;
extern volatile float U_current, V_current, W_current;
extern volatile float I_q_avg, I_d_avg;

extern volatile float sin_elec_position, cos_elec_position;
extern volatile float I_a, I_b, I_q, I_d;
extern volatile float cmd_q, cmd_d;
extern volatile float cmd_a, cmd_b;
extern volatile float integ_q, integ_d;
extern volatile float Kp_Iq, Ki_Iq;
extern volatile float Kp_Id, Ki_Id;
extern volatile float I_d_err, I_q_err;
extern volatile float TargetCurrent;
extern volatile float TargetFieldWk;
extern volatile uint32_t F_sw;

extern volatile const uint8_t SVPWM_PermuataionMatrix[6][3];
extern volatile uint8_t	SVPWM_sector;
extern volatile float SVPWM_mag, SVPWM_ang;
extern volatile float SVPWM_Ti[4];
extern volatile float SVPWM_Tb1, SVPWM_Tb2;
extern volatile float SVPWM_beta;
extern volatile float duty_u, duty_v, duty_w;

extern volatile uint8_t sendCANBus_flag;

extern void FOC();
extern void writePwm(uint32_t timer, int32_t duty);
extern int32_t lookupTbl(const int32_t* source, const int32_t* target, const uint32_t size, const int32_t value);
extern float lookupTblf(const float* source, const float* target, const uint32_t size, const float value);
extern float flookupTbll(const int32_t* source, const float* target, const uint32_t size, const int32_t value);
extern const float sin_LookupX[];
extern const int32_t sin_LookupXl[];
extern const float sin_LookupY[];
extern const float cos_LookupY[];
extern const uint16_t math_LookupSize;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1);
  x = LL_SPI_ReceiveData16(SPI1);
  switch (SPI_WaitState)
  {
  case 0: // prepare to recieve 32 bits
    motor_lastMeasTime2 = TIM2->CNT - 4;
    ENDAT_DIR_Read;
    LL_SPI_Disable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
    LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_LSB_FIRST);
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
    LL_SPI_Enable(SPI1);
    LL_SPI_TransmitData16(SPI1, 0U);
    LL_SPI_TransmitData16(SPI1, 0U);
    SPI_WaitState = 1;
    break;
  case 1: // first 16 bits done
    SPI_buf |= x;
    SPI_WaitState = 2;
    break;
  case 2: // second 16 bits done
    SPI_buf |= x << 16;
    SPI_WaitState = 3;
    SPI_Wait = false;
    break;
  default:
    SPI_WaitState = 0;
    break;
  }
  /* USER CODE END SPI1_IRQn 0 */
  /* USER CODE BEGIN SPI1_IRQn 1 */
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1);
  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt, DAC2 and DAC4 channel underrun error interrupts.
  */
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */
  LL_TIM_ClearFlag_UPDATE(TIM7);
  sendCANBus_flag = 4;
  /* USER CODE END TIM7_DAC_IRQn 0 */

  /* USER CODE BEGIN TIM7_DAC_IRQn 1 */

  /* USER CODE END TIM7_DAC_IRQn 1 */
}

/**
  * @brief This function handles HRTIM timer B global interrupt.
  */
void HRTIM1_TIMB_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMB_IRQn 0 */
  LL_HRTIM_ClearFlag_UPDATE(HRTIM1, LL_HRTIM_TIMER_B);
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);
  
  FOC();

  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
  /* USER CODE END HRTIM1_TIMB_IRQn 0 */

  /* USER CODE BEGIN HRTIM1_TIMB_IRQn 1 */

  /* USER CODE END HRTIM1_TIMB_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
