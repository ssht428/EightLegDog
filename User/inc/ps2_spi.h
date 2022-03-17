/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File：PS2驱动程序
Author：pinggai    Version:1.0     Data:2015/05/16
Description: PS2驱动程序
**** 2020.10.22: V1.1: use spi to drive ps2 joystick,max frequency<312.5khz
**********************************************************/	 


#ifndef __PS2_SPI_H
#define __PS2_SPI_H

#include "stm32l1xx.h"

/* Exported macro ---------------------------------------------*/
/* PS2 connect to SPI4, VCC->3V,
 *  DI/DAT <-> PE5(SPI4_MISO)    , DO/CMD <-> PE6(SPI4_MOSI) , 
 *  CS <-> PE4(SPI4_NSS),  CLK <-> PE2(SPI4_SCK) */
#define PS2_SPI                              SPI2
#define PS2_SPI_CLK_ENABLE()                 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE)
#define PS2_SPI_GPIO_CLK_ENABLE()            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE)
#define PS2_SPI_GPIO_PORT                    GPIOB
#define PS2_SPI_GPIO_PIN                     (GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15)
#define PS2_SPI_SCK_PIN                      GPIO_Pin_13
#define PS2_SPI_MISO_PIN                     GPIO_Pin_14
#define PS2_SPI_MOSI_PIN                     GPIO_Pin_15
#define PS2_SPI_SCK_PINSOURCE                GPIO_PinSource13
#define PS2_SPI_MISO_PINSOURCE               GPIO_PinSource14
#define PS2_SPI_MOSI_PINSOURCE               GPIO_PinSource15
#define PS2_SPI_AF                           GPIO_AF_SPI2

#define PS2_CS_PORT                          GPIOB
#define PS2_CS_CLK_ENABLE()                  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE)
#define PS2_CS_PIN                           GPIO_Pin_12
#define PS2_SPI_CS_RESET()                   (PS2_CS_PORT->BSRRH = PS2_CS_PIN)
#define PS2_SPI_CS_SET()                     (PS2_CS_PORT->BSRRL = PS2_CS_PIN)

#define  PS2_STATE_NOCHANGE					((unsigned int)0)			/* clear  point */
#define  PS2_STATE_CHANGED					((unsigned int)1)			/* fill point with color */
#define  PS2_SCAN_TICK							((unsigned int)5)			/* unit=systick, scan cycle time */
#define  PS2_DEBOUNCE_TICK					((unsigned int)3)			/* unit=systick,only after x-systick then the key is valid */
#define  PS2_LONG_TICK							((unsigned int)10)			/* unit=scan cycle times, */
#define  PS2_HOLD_TICK							((unsigned int)20)			/* unit=scan cycle times, */

#define  PS2_KEY_NUM  									16                      //定义按键总数
#define  PS2_KEY_VALUE_MASK							((unsigned int)0xFFFF)		/* bit0~15 represent key value */
#define  PS2_KEY_STATE_MASK             ((unsigned int)0xF0000)		/* bit16~19 represent key press state */
#define  PS2_KEY_SHORT_MASK							((unsigned int)0x10000)		/* bit16 represent Short-press */
#define  PS2_KEY_LONG_MASK							((unsigned int)0x20000)		/* bit17 represent Long-press */
#define  PS2_KEY_HOLD_MASK							((unsigned int)0x40000)		/* bit18 represent Hold-press */


//These are our button constants,which return through spi,Data[3]<<8 &Data[4]
#define PSB_NONE        0xFFFF
#define PSB_SELECT      0xFEFF
#define PSB_L3          0xFDFF
#define PSB_R3          0xFBFF
#define PSB_START       0xF7FF
#define PSB_UP          0xEFFF
#define PSB_RIGHT       0xDFFF
#define PSB_DOWN        0xBFFF
#define PSB_LEFT        0x7FFF
#define PSB_L1          0xFFFB
#define PSB_L2          0xFFFE
#define PSB_R1          0xFFF7
#define PSB_R2          0xFFFD
#define PSB_GREEN       0xFFEF
#define PSB_RED         0xFFDF
#define PSB_BLUE        0xFFBF
#define PSB_PINK        0xFF7F
#define PSB_TRIANGLE    0xFFEF
#define PSB_CIRCLE      0xFFDF
#define PSB_CROSS       0xFFBF
#define PSB_SQUARE      0xFF7F

//These are stick values in digital mode
#define PSS_R_UP          PSB_TRIANGLE
#define PSS_R_DOWN        PSB_CROSS
#define PSS_R_LEFT        PSB_SQUARE
#define PSS_R_RIGHT       PSB_CIRCLE
#define PSS_L_UP          PSB_UP              
#define PSS_L_DOWN        PSB_DOWN
#define PSS_L_LEFT        PSB_LEFT
#define PSS_L_RIGHT       PSB_RIGHT

void PS2_Init(void);
uint16_t PS2_GetKey(void);
unsigned int PS2_KeyScan(void);		       

#endif  // __PS2_SPI_H





