/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "stm32l1_discovery.h"
//#include "stm32l1_discovery_lcd.h"
#include "stm32l1xx_it.h"
#include "ps2_spi.h"
#include "mbcrc.h"
/* Exported typedef -----------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Exported define ------------------------------------------------------------*/
#define TIMEOUT_PS2                     500          // unit=systick

/* Uncomment the line below if you will use the USART in Transmitter Mode */
 #define USART_TRANSMITTER 
/* Uncomment the line below if you will use the USART in Receiver Mode */
/* #define USART_RECEIVER   */
 
#define USARTx                           USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler
 
#define USARTx_TX_PIN                    GPIO_Pin_9                
#define USARTx_TX_GPIO_PORT              GPIOA                       
#define USARTx_TX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource9
#define USARTx_TX_AF                     GPIO_AF_USART1
 
#define USARTx_RX_PIN                    GPIO_Pin_10                
#define USARTx_RX_GPIO_PORT              GPIOA                    
#define USARTx_RX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource10
#define USARTx_RX_AF                     GPIO_AF_USART1

#define USARTx_DE_PIN                    GPIO_Pin_11                
#define USARTx_DE_GPIO_PORT              GPIOA                    
#define USARTx_DE_GPIO_CLK               RCC_AHBPeriph_GPIOA

#define	USARTx_TX_EN	                   (USARTx_DE_GPIO_PORT->BSRRL = USARTx_DE_PIN)        //GPIO_SetBits(USARTx_DE_GPIO_PORT, USARTx_DE_PIN)	 	
#define	USARTx_RX_EN	                   (USARTx_DE_GPIO_PORT->BSRRH = USARTx_DE_PIN)        //GPIO_ResetBits(USARTx_DE_GPIO_PORT, USARTx_DE_PIN)	

#define BUFFERSIZE                        20
#define MAX_RX_ERROR                      10                // maximum time of slave response error
#define MAX_MOTOR_SPEED                   (uint16_t)500     // maximum speed of wheel(0~1000),unit=0.1hz
#define MIN_MOTOR_SPEED                   (uint16_t)65036   // minimum speed of wheel(64536~65536),unit=0.1hz
#define MAX_PS2_TIMEOUT                   80                // unit= scan cycle time
#define LAST_MOVE_STOP                    PSB_NONE
#define LAST_MOVE_FORWARD                 PSB_UP
#define LAST_MOVE_BACKWARD                PSB_DOWN
#define LEG_ID_RIGHT                      0x01
#define LEG_ID_LEFT                       0x03
#define MOVE_MODE_SPEED                   0
#define MOVE_MODE_POSITION                1
#define LED_POSITION_MODE                 LED3
#define LED_SPEED_MODE                    LED4
#define MIN_MBCMD_DELAY                   5                 // unit = scan cycle time

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint8_t cmd[20];
  uint8_t len;

}MbCommand_Structure;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
