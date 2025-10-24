/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define BUFFER_SIZE  3600
//#define HALF_SIZE    (BUFFER_SIZE / 2)

//extern uint8_t buffer1[BUFFER_SIZE];

// 使用 volatile 来防止编译器优化
//volatile float angle_array[360];   // 存储计算出的角度
//volatile float distance_array[380]; // 存储计算出的距离
extern volatile int flag;
extern volatile int tip;
extern int q=0;
//extern  uint8_t tx_buffer[2600]; 
volatile int r=1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
 if (__HAL_DMA_GET_FLAG(&hdma_usart3_rx, DMA_FLAG_TCIF2_6)){
// tip=1;
	 //OLED_ShowNum8X16(0,2,tip,8);
	 __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, DMA_FLAG_TCIF2_6);
 }
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
if (__HAL_DMA_GET_FLAG(&hdma_uart4_rx, DMA_FLAG_HTIF2_6))
      {    flag=1;
				//OLED_ShowNum8X16(0,4,q++,8);
					HAL_TIM_Base_Start_IT(&htim5);
			// OLED_ShowNum8X16(0,4,q++,8);
       //__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TC);  // 禁用传输完成中断
      //__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);  // 禁用半传输完成中断
      // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TE);  // 禁用传输错误中断

        // 处理前半段数据
			 // int j=0;
     //   for (uint16_t i = 0; i < HALF_SIZE; i += 5)   // 这里根据你的数据帧长度修改步进
      //  {  
      //      float angle_new = (((uint16_t)(buffer1[i + 2] << 7) + (buffer1[i + 1] >> 1))) / 64.0f;
      //      float distance_new = (((uint16_t)(buffer1[i + 4] << 8) + (buffer1[i + 3]))) / 4.0f;
      //       angle_array[j++]=angle_new;   // 存储计算出的角度
      //       distance_array[j++]=distance_new;
    //    }
         
	  
				// __HAL_DMA_CLEAR_FLAG(&hdma_uart4_rx, DMA_FLAG_HTIF2_6);
			
    }

    // ==== 全部传输完成（后半段数据搬运完） ====
  //  if (__HAL_DMA_GET_FLAG(&hdma_uart4_rx, DMA_FLAG_TCIF2_6))
   // {   // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TC);  // 禁用传输完成中断
     // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);  // 禁用半传输完成中断
      // __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TE);  // 禁用传输错误中断

        // 处理后半段数据
			//int j=0;
      //  for (uint16_t i = HALF_SIZE; i < BUFFER_SIZE; i += 5)
      //  {   
      //      float angle_new = (((uint16_t)(buffer1[i + 2] << 7) + (buffer1[i + 1] >> 1))) / 64.0f;
      //      float distance_new = (((uint16_t)(buffer1[i + 4] << 8) + (buffer1[i + 3]))) / 4.0f;
      //      angle_array[j++]=angle_new;   // 存储计算出的角度
      //     distance_array[j++]=distance_new;
           
     //   }

        // 当一个360点循环完成后，可以清零索引（根据实际需求）
       // __HAL_DMA_CLEAR_FLAG(&hdma_uart4_rx, DMA_FLAG_TCIF2_6);
				// __HAL_DMA_CLEAR_FLAG(&hdma_uart4_rx, DMA_FLAG_HTIF2_6);
				//__HAL_DMA_ENABLE_IT(&hdma_uart4_rx, DMA_IT_TC);  // 恢复传输完成中断
       // __HAL_DMA_ENABLE_IT(&hdma_uart4_rx, DMA_IT_HT);  // 恢复半传输完成中断
       //__HAL_DMA_ENABLE_IT(&hdma_uart4_rx, DMA_IT_TE);  // 恢复传输错误中断
			
		//	OLED_ShowString8X16(0,6,"3456");		
	//	}
  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */
	//OLED_ShowString8X16(0,6,"3456");	
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TC);  // 禁用传输完成中断
 __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);  // 禁用半传输完成中断
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_TE);  // 禁用传输错误中断

    // 更新DMA索引
  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
/*if (__HAL_DMA_GET_FLAG(&hdma_usart3_tx, DMA_FLAG_TCIF2_6)){
/*if(r<13){ 
	HAL_UART_Transmit_DMA(&huart3, tx_buffer+r*200, 200);  
r++;
}

	if(r==13){r=0;}
	
	 __HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_FLAG_TCIF2_6);}*/
	
	
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
