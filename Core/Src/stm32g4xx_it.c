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
extern volatile int32_t dt_i;

extern volatile int16_t adc_data[4];
extern volatile int16_t adc_os[3];

extern volatile bool SPI_Wait;
extern volatile uint8_t SPI_WaitState;
extern volatile uint32_t SPI_buf;

extern volatile float motor_ElecPosition;
extern volatile int32_t motor_PhysPosition;
extern volatile int32_t Encoder_os;
extern volatile uint32_t motor_lastMeasTime;
extern volatile float motor_speed;
extern volatile float U_current, V_current, W_current;

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

extern volatile const uint8_t SVPWM_PermuataionMatrix[6][3];
extern volatile uint8_t	SVPWM_sector;
extern volatile float SVPWM_mag, SVPWM_ang;
extern volatile float SVPWM_Ti[4];
extern volatile float SVPWM_Tb1, SVPWM_Tb2;
extern volatile float SVPWM_beta;
extern volatile float duty_u, duty_v, duty_w;

extern volatile uint8_t sendCANBus_flag;

extern void writePwm(uint32_t timer, int32_t duty);
extern int32_t lookupTbl(const int32_t* source, const int32_t* target, const uint32_t size, const int32_t value);
extern float lookupTblf(const float* source, const float* target, const uint32_t size, const float value);
extern const float sin_LookupX[];
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
    motor_lastMeasTime = TIM2->CNT - 4;
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
  * @brief This function handles HRTIM master timer global interrupt.
  */
void HRTIM1_Master_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_Master_IRQn 0 */
  LL_HRTIM_ClearFlag_REP(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);
  // convert ADC values to phase current
  U_current = (adc_data[0] - adc_os[0]) * 0.075f;
  V_current = (adc_data[1] - adc_os[1]) * 0.075f;
  W_current = (adc_data[2] - adc_os[2]) * 0.075f;
  // FOC
  motor_ElecPosition = fmodf(((float)(motor_PhysPosition + Encoder_os) + (TIM2->CNT - motor_lastMeasTime)*motor_speed)
    * (float)N_POLES / (float)N_STEP_ENCODER, 1.0f);
  sin_elec_position = lookupTblf(sin_LookupX, sin_LookupY, math_LookupSize, motor_ElecPosition);
  cos_elec_position = lookupTblf(sin_LookupX, cos_LookupY, math_LookupSize, motor_ElecPosition);
  // sin_elec_position = sinf(motor_ElecPosition * PIx2);
  // cos_elec_position = cosf(motor_ElecPosition * PIx2);
  // Clarke transform
  I_a = U_current * 0.66666667f - V_current * 0.33333333f - W_current * 0.33333333f;
  I_b = sqrt3_1o * (V_current - W_current);
  // Park transform
  I_d = I_a * cos_elec_position + I_b * sin_elec_position;
  I_q = I_b * cos_elec_position - I_a * sin_elec_position;
  // PI controllers on Q and D
  I_d_err = TargetFieldWk - I_d;
  I_q_err = TargetCurrent - I_q;
  integ_d += I_d_err * Ki_Id * 25e-6f;
  integ_d = (integ_d > MAX_CMD_D) ? MAX_CMD_D : integ_d;
  integ_d = (integ_d < MIN_CMD_D) ? MIN_CMD_D : integ_d;
  cmd_d = I_d_err * Kp_Id + integ_d;
  integ_q += I_q_err * Ki_Iq * 25e-6f;
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
  // arm_sqrt_f32(cmd_a*cmd_a + cmd_b*cmd_b, &SVPWM_mag);
  // SVPWM_mag = hypotf(cmd_b, cmd_a);
  // // arm_atan2_f32(cmd_b, cmd_a, &SVPWM_ang);
  // SVPWM_ang = atan2f(cmd_b, cmd_a);
  // SVPWM_ang += PI;
  // SVPWM_mag = (SVPWM_mag > sqrt3_1o) ? sqrt3_1o : SVPWM_mag;
  // SVPWM_sector = (uint8_t)(SVPWM_ang * PI_3o);
  // SVPWM_beta = SVPWM_ang - PIo3 * SVPWM_sector;
  // SVPWM_Tb1 = SVPWM_mag * sinf(PIo3 - SVPWM_beta);
  // SVPWM_Tb2 = SVPWM_mag * sinf(SVPWM_beta);
  // SVPWM_Ti[0] = (1.0f - SVPWM_Tb1 - SVPWM_Tb2) * 0.5f;
  // SVPWM_Ti[1] = SVPWM_Tb1 + SVPWM_Tb2 + SVPWM_Ti[0];
  // SVPWM_Ti[2] = SVPWM_Tb2 + SVPWM_Ti[0];
  // SVPWM_Ti[3] = SVPWM_Tb1 + SVPWM_Ti[0];
  // Update duty cycle
  // duty_u = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][0]];
  // duty_v = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][1]];
  // duty_w = SVPWM_Ti[SVPWM_PermuataionMatrix[SVPWM_sector][2]];
  // writePwm(U_TIMER, duty_u * 64000.0f);
  // writePwm(V_TIMER, duty_v * 64000.0f);
  // writePwm(W_TIMER, duty_w * 64000.0f);
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
  /* USER CODE END HRTIM1_Master_IRQn 0 */

  /* USER CODE BEGIN HRTIM1_Master_IRQn 1 */

  /* USER CODE END HRTIM1_Master_IRQn 1 */
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
