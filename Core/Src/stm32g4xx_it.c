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
#include "FOC.h"
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
extern void disableGateDriver();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan2;
/* USER CODE BEGIN EV */
extern volatile bool SPI_Wait;
extern volatile uint8_t SPI_WaitState;
extern volatile uint32_t SPI_buf;
extern volatile uint32_t motor_lastMeasTime2; // 2 is for saving data temporarity before moving it to 1

extern volatile FOC_data* FOC;

extern volatile uint8_t sendCANBus_flag;
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
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1);
  x = SPI1->DR;//LL_SPI_ReceiveData16(SPI1);
  switch (SPI_WaitState)
  {
  case 0: // prepare to recieve 32 bits
    ENDAT_DIR_Read;
    // LL_SPI_Disable(SPI1);
    SPI1->CR1 &= ~SPI_CR1_SPE; // disable SPI
    // LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
    // LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_LSB_FIRST);
    // LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
    SPI1->CR1 |= SPI_CR1_LSBFIRST | SPI_CR1_CPHA; // LSB first, CPHA = 1
    SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3; // 16 bits
    // LL_SPI_Enable(SPI1);
    SPI1->CR1 |= SPI_CR1_SPE; // enable SPI
    // LL_SPI_TransmitData16(SPI1, 0U);
    // LL_SPI_TransmitData16(SPI1, 0U);
    SPI1->DR = 0U; // send dummy data
    SPI1->DR = 0U; // send dummy data
    SPI_WaitState = 1;
    motor_lastMeasTime2 = TIM2->CNT - 5;
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
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  LL_TIM_ClearFlag_UPDATE(TIM6);
  disableGateDriver();
  /* USER CODE END TIM6_DAC_IRQn 0 */

  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
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
  
  FOC_update(FOC);

  
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
