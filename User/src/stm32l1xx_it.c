/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/stm32l1xx_it.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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
#include "stm32l1xx_it.h"
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
extern __IO IT_SystickTypeDef gSystick;
extern uint8_t aTxBuffer[];
extern uint8_t aRxBuffer[];
extern __IO uint8_t ubRxIndex;
extern __IO uint8_t ubTxIndex;
extern __IO uint8_t ubTxSize;
extern __IO uint8_t ubRxError;
extern __IO uint32_t TimeOut;



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
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
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
      HAL_IncTick();
  
      /* Cyclick reset timer flag for few software tick task */
      ++gSystick.tick;
      SYSTICK_TIMER = 1;
      gSystick.timer.task |= 0x0000000F;
      if (0 == (gSystick.tick % 2))
      {
          SYSTICK_TIMER2 = 1;
          gSystick.timer.task |= 0x000000F0;
      }    if (0 == (gSystick.tick % 5))
      {
          SYSTICK_TIMER5 = 1;
          gSystick.timer.task |= 0x00000F00;
      }
      if (0 == (gSystick.tick % 10))
      {
          SYSTICK_TIMER10 = 1;
          gSystick.timer.task |= 0x0000F000;
      }
      if (0 == (gSystick.tick % 20))
      {
          SYSTICK_TIMER20 = 1;
          gSystick.timer.task |= 0x000F0000;
      }    if (0 == (gSystick.tick % 50))
      {
          SYSTICK_TIMER50 = 1;
          gSystick.timer.task |= 0x00F00000;
      }
      if (0 == (gSystick.tick % 100))
      {
          SYSTICK_TIMER100 = 1;
          gSystick.timer.task |= 0x0F000000;
      }
      if (0 == (gSystick.tick % 200))
      {
          SYSTICK_TIMER200 = 1;
          gSystick.timer.task |= 0xF0000000;
      }    
      if (gSystick.tick >= 200)
      {
          gSystick.tick = 0;
      }
    
}
/*--------------------------------------------------------------
     Name       : void IT_ResetSystickBit( void )
     Description: Reset systick timer bit to prevent further task interrupt
     Parameter  : (None)
     Returns    : (None)
   ----------------------------------------------------------------*/
 void IT_ResetSystickBit(void)
 {
 
     gSystick.timer.task = 0;
     gSystick.status.par = 0;
     SYSTICK_TIMER = 0;
     SYSTICK_TIMER2 = 0;
     SYSTICK_TIMER5 = 0;
     SYSTICK_TIMER10 = 0;
     SYSTICK_TIMER20 = 0;
     SYSTICK_TIMER50 = 0;
     SYSTICK_TIMER100 = 0;
     SYSTICK_TIMER200 = 0;
 }

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
void USARTx_IRQHandler(void)
{
  /* USART in Receiver mode */
  if (USART_GetITStatus(USARTx, USART_IT_RXNE) == SET)
  {
    if(ubRxIndex>=BUFFERSIZE)
      ubRxIndex = 0;
    /* Receive Transaction data */
    aRxBuffer[ubRxIndex++] = USART_ReceiveData(USARTx);
    USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
     /* check receive function code,if slave error,then return (function code | 0x80) */
    if(ubRxIndex>1)      
    {
      if(aRxBuffer[1] > 0x80)
      {
        ubRxError++;
        if(ubRxError > MAX_RX_ERROR)
        {
          /* display error info on LCD, or stop motors  */
        }
      }
      else
        ubRxError = 0;
    }
  }

}

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
