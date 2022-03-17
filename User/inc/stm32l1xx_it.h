/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/stm32l1xx_it.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32L1xx_IT_H
#define __STM32L1xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    unsigned short bit0: 1;
    unsigned short bit1: 1;
    unsigned short bit2: 1;
    unsigned short bit3: 1;
    unsigned short bit4: 1;
    unsigned short bit5: 1;
    unsigned short bit6: 1;
    unsigned short bit7: 1;
    unsigned short bit8: 1;
    unsigned short bit9: 1;
    unsigned short bit10: 1;
    unsigned short bit11: 1;
    unsigned short bit12: 1;
    unsigned short bit13: 1;
    unsigned short bit14: 1;
    unsigned short bit15: 1;    
} IT_SystickStatusBit;

typedef union
{
    unsigned short par;
    IT_SystickStatusBit field;
} IT_SystickStatus;

typedef struct
{
    unsigned int bit0: 4;   // 1systick
    unsigned int bit4: 4;   // 2systick
    unsigned int bit8: 4;   // 5systick
    unsigned int bit12: 4;  // 10systick
    unsigned int bit16: 4;  // 20systick
    unsigned int bit20: 4;  // 50systick
    unsigned int bit24: 4;  // 100systick
    unsigned int bit28: 4;  // 200systick
} IT_SystickTimerBit;

typedef union
{
    unsigned int task;
    IT_SystickTimerBit field;
} IT_SystickTimer;

typedef struct
{
    unsigned int tick;       /* counter of systick,base interval of systick,1ms or 10ms */
    /* each 4bit represent 1,2,5,10,20,50,100,200  */
    IT_SystickTimer timer;
    IT_SystickStatus  status;
} IT_SystickTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define     SYSTICK_TIMER            (gSystick.status.field.bit0)
#define     SYSTICK_TIMER2           (gSystick.status.field.bit1)
#define     SYSTICK_TIMER5           (gSystick.status.field.bit2)
#define     SYSTICK_TIMER10          (gSystick.status.field.bit3)
#define     SYSTICK_TIMER20          (gSystick.status.field.bit4)
#define     SYSTICK_TIMER50          (gSystick.status.field.bit5)
#define     SYSTICK_TIMER100         (gSystick.status.field.bit6)
#define     SYSTICK_TIMER200         (gSystick.status.field.bit7)
#define     SYSTICK_TMPTIMER         (gSystick.status.field.bit8)
#define     SYSTICK_TMPTIMER2        (gSystick.status.field.bit9)
#define     SYSTICK_TMPTIMER5        (gSystick.status.field.bit10)
#define     SYSTICK_TMPTIMER10       (gSystick.status.field.bit11)
#define     SYSTICK_TMPTIMER20       (gSystick.status.field.bit12)
#define     SYSTICK_TMPTIMER50       (gSystick.status.field.bit13)
#define     SYSTICK_TMPTIMER100      (gSystick.status.field.bit14)
#define     SYSTICK_TMPTIMER200      (gSystick.status.field.bit15)

/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void IT_ResetSystickBit(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L1xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
