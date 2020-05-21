/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Socket_RTOS/Src/stm32f7xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_it.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern ETH_HandleTypeDef EthHandle;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  osSystickHandler();
}

/******************************************************************************/
/*                 STM32F7xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f7xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles Ethernet interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
  HAL_ETH_IRQHandler(&EthHandle);
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}


/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
    uint16_t bytes_left = 0;
    if(READ_BIT(huart6.Instance->ISR, USART_ISR_IDLE))
    {
        SET_BIT(huart6.Instance->ICR, USART_ICR_IDLECF);
        bytes_left = huart6.hdmarx->Instance->NDTR;
        
        if((bytes_left < RX_BUF_SLICE_SIZE) && (bytes_left > 0))
        {
            USART6_ISR_IdleLine_Callback(&huart6, bytes_left);
        }
    }
    if(READ_BIT(huart6.Instance->ISR, USART_ISR_ORE))
    {
        SET_BIT(huart6.Instance->ICR, USART_ICR_ORECF);
    }

    
    /*
    if(READ_BIT(huart6.Instance->ISR, USART_ISR_RTOF))
    {
        SET_BIT(huart6.Instance->ICR, USART_ICR_RTOCF);
        byte_left = huart6.hdmarx->Instance->NDTR;
        if(byte_left)
        {
            byte_left++;
        }
    }*/
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
