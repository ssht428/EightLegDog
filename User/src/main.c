/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   Main program body
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
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */
/* Public variables -which is also used in other c files------------------------*/
__IO IT_SystickTypeDef  gSystick;

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define USE_PRINTF_DEBUG    0

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t curTick;
__IO uint16_t btnPress;
__IO uint32_t curKey = 0xFFFF;    // current ps2 key
__IO uint32_t curKeyValue = PSB_NONE;     // = curKey & 0xFFFF
__IO uint32_t curKeyState = 0;            // = curKey & 0xF0000
__IO uint32_t lastKeyValue = 0xFFFF;   // last ps2 key,include PSB_NONE
__IO uint32_t oldKeyValue = 0xFFFF;    // old valid key
__IO uint32_t lastFwdBkd = LAST_MOVE_STOP;  // store last pressed key of wheel direction
__IO uint32_t ps2KeyTimeout = 0;               // time of ps2 no button is pressed 
__IO int flagLeftLegRun = LAST_MOVE_STOP;                // flag of left leg is running
__IO int flagRightLegRun = LAST_MOVE_STOP;                // flag of right leg is running
__IO int flagEnableDisable = 0;               // 1=motor driver is enabled,0=disabled

uint8_t aTxBuffer[BUFFERSIZE] = {0x00,0x06,0x00,0x43,0x00,0x60};
uint8_t aRxBuffer [BUFFERSIZE];
__IO uint8_t ubRxIndex = 0x00;
__IO uint8_t ubTxIndex = 0x00;
__IO uint8_t ubTxSize = 6;
__IO uint8_t ubRxError = 0;
__IO uint32_t TimeOut = 0x00;  
uint8_t bSend = 0;

__IO uint32_t curLegSpeedLeft = 65436;         // left leg speed(65536-x),unit=0.1rps
__IO uint32_t curLegSpeedRight = 100;          // right leg speed,unit=0.1rps
int initializing = 1;
int stateInit = 0;
int moveMode = MOVE_MODE_SPEED;

int stopPS2 = 0;            // stop receive new ps2 key for continue current operation
static __IO int32_t curPositionLeft = 0;
static __IO int32_t curPositionRight = 0;

static __IO uint32_t mbCmdDelay = 0;



/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
#if(USE_PRINTF_DEBUG)
  #include <stdio.h>
    /** 
     * @brief private impleted printf function,this would save a lot
     *            of rom&ram space compare to the built in printf
     *            function in the C newlib.Carefully use it inside a
     *            interrupt service routine.
     * @param fmt
     * 
     * @return int
     */
    int fputc(int ch, FILE *f)
    {
        return(ITM_SendChar(ch));
    }
#endif
// leg motor driver command 
MbCommand_Structure mbCmdNop =      // incase first byte isn't send out after MCU initialize 
{
  {0x00,0x03,0x60,0x41,0x00,0x01,0xCB,0xCF},
  8
};
MbCommand_Structure mbCmdDisable =    // 0x6040's bit6=1 means releative move,no action in speed mode
{
  {0x00,0x06,0x60,0x40,0x00,0x00,0x97,0xCF},
  8
};  
MbCommand_Structure mbCmdInitial =    // 0x6040's bit6=1 means releative move,no action in speed mode
{
  {0x00,0x06,0x60,0x40,0x00,0x4F,0xD6,0x3B},
  8
};
MbCommand_Structure mbCmdPosMode = 
{
  {0x00,0x06,0x60,0x60,0x00,0x01,0x57,0xC5},
  8
};
MbCommand_Structure mbCmdPosSpeedAccDec =   // position mode:speed,acceleration,deceleration setting
{
 //addr,func, reg addr,    reg num,   bytes, target speed,      acc,       dec,       CRC
  {0x00,0x10,0x60,0x81,0x00,0x04,0x08,0x00,0x00,0x00,0x64,0x00,0x64,0x00,0x64,0x38,0x9B},
  17
};
MbCommand_Structure mbCmdSpeedMode =    // default mode
{
  {0x00,0x06,0x60,0x60,0x00,0x03,0xD6,0x04},
  8
};  
MbCommand_Structure mbCmdSpeedAccDecLeft = 
{
 //addr,func, reg addr,    reg num,   bytes, target speed,      acc,       dec,       CRC
  {0x03,0x10,0x60,0x81,0x00,0x04,0x08,0xFF,0xFF,0xFF,0x9C,0x00,0x64,0x00,0x64,0x8E,0x4A},
  17
};
MbCommand_Structure mbCmdSpeedAccDecRight = 
{
 //addr,func, reg addr,    reg num,   bytes, target speed,      acc,       dec,       CRC
  {0x01,0x10,0x60,0x81,0x00,0x04,0x08,0x00,0x00,0x00,0x64,0x00,0x64,0x00,0x64,0xF9,0x9B},
  17
}; 
MbCommand_Structure mbCmdSpeedLeft = 
{
 //addr,func, reg addr,    reg num,   bytes, target speed,      CRC
  {0x03,0x10,0x60,0x81,0x00,0x02,0x04,0xFF,0xFF,0xFF,0x9C,0xD8,0x04},
  13
};
MbCommand_Structure mbCmdSpeedRight = 
{
 //addr,func, reg addr,    reg num,   bytes, target speed,       CRC
  {0x01,0x10,0x60,0x81,0x00,0x02,0x04,0x00,0x00,0x00,0x64,0x93,0xEA},
  13
};  
MbCommand_Structure mbCmdPosLeft =  // position mode : forward, fixed at 96pulse per movement 
{
 //addr,func, reg addr,    reg num,   bytes, relative position,     CRC
  {0x03,0x10,0x60,0x7A,0x00,0x02,0x04,0xFF,0xFF,0xF0,0x00,0x93,0x6A},
  13 
};
MbCommand_Structure mbCmdPosRight = 
{
  {0x01,0x10,0x60,0x7A,0x00,0x02,0x04,0x00,0x00,0x0F,0x00,0xD9,0x06},
  13   
};
  // downside is speed mode control command
MbCommand_Structure mbCmdSpeedStart = 
{
  {0x00,0x06,0x60,0x40,0x00,0x0F,0xD7,0xCB},
  8
};
MbCommand_Structure mbCmdSpeedStartLeft = 
{
  {0x03,0x06,0x60,0x40,0x00,0x0F,0xD7,0xF8},
  8
};
MbCommand_Structure mbCmdSpeedStartRight = 
{
  {0x01,0x06,0x60,0x40,0x00,0x0F,0xD6,0x1A},
  8
};  
MbCommand_Structure mbCmdSpeedStop = 
{
  {0x00,0x06,0x60,0x40,0x01,0x0F,0xD6,0x5B},
  8
};
MbCommand_Structure mbCmdSpeedStopLeft = 
{
  {0x03,0x06,0x60,0x40,0x01,0x0F,0xD6,0x68},
  8
};
MbCommand_Structure mbCmdSpeedStopRight = 
{
  {0x01,0x06,0x60,0x40,0x01,0x0F,0xD7,0x8A},
  8
};
   // downside is position mode control command
MbCommand_Structure mbCmdPosStart = 
{
  {0x00,0x06,0x60,0x40,0x00,0x5F,0xD7,0xF7},
  8
};
MbCommand_Structure mbCmdPosStartLeft = 
{
  {0x03,0x06,0x60,0x40,0x00,0x5F,0xD7,0xC4},
  8
};
MbCommand_Structure mbCmdPosStartRight = 
{
  {0x01,0x06,0x60,0x40,0x00,0x5F,0xD6,0x26},
  8
};  
MbCommand_Structure mbCmdPosStop = 
{
  {0x00,0x06,0x60,0x40,0x01,0x4F,0xD7,0xAB},
  8
};
MbCommand_Structure mbCmdPosStopLeft = 
{
  {0x03,0x06,0x60,0x40,0x01,0x4F,0xD7,0x98},
  8
};
MbCommand_Structure mbCmdPosStopRight = 
{
  {0x01,0x06,0x60,0x40,0x01,0x4F,0xD6,0x7A},
  8
};




/* Private function prototypes -----------------------------------------------*/
/*
 * brief: send command to motor driver.
 * pcmd: pointer to  command array,
 * return: response correct or wrong
*/
uint8_t mainMbSendCmd(MbCommand_Structure *pcmd)
{
  int i = 0;

  ubRxIndex = 0;
  for(i=0;i<BUFFERSIZE;i++)
    aRxBuffer[i] = 0;
  USARTx_TX_EN;
  HAL_Delay(0);
  for(i=0;i<pcmd->len;i++)
  {
    USART_SendData(USARTx,pcmd->cmd[i]);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){};
  }
  HAL_Delay(0);
  USARTx_RX_EN;
  HAL_Delay(5);
  return ubRxError;
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK | USARTx_DE_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USARTx_DE_PIN; 		
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(USARTx_DE_GPIO_PORT, &GPIO_InitStructure);	
  
  USARTx_TX_EN;
  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follows:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - two Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */ 
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_2;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

/* Automatic Initialization */
void mainAutoInit(void)
{
  while(initializing)
  {
    /*******************      50systick(500ms) scan cycle task ***********/
    if(1 == SYSTICK_TIMER20)        
    {
      switch(stateInit)
      {
        case 0:   // send nop command incase first byte is droped by UART
            mainMbSendCmd(&mbCmdNop);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED4);
            stateInit = 1;
        break;
        case 1:   // Initialize mbCmding motor driver
            mainMbSendCmd(&mbCmdInitial);
            HAL_Delay(100);
            
            if(moveMode == MOVE_MODE_POSITION)
            {
              STM_EVAL_LEDOn(LED_POSITION_MODE);
              HAL_Delay(100);
              stateInit = 2;
            }
            else if(moveMode == MOVE_MODE_SPEED)
            {           
              STM_EVAL_LEDOn(LED_SPEED_MODE);
              HAL_Delay(100);
              stateInit = 10;
            }
        break;

        /* Position mode setting  */
        case 2:   // chose position mode
            mainMbSendCmd(&mbCmdPosMode);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED_POSITION_MODE);
            stateInit = 3;
        break;
        case 3:   // position mode's speed,acc,dec setting
            mainMbSendCmd(&mbCmdPosSpeedAccDec);
            HAL_Delay(100);
            STM_EVAL_LEDOn(LED_POSITION_MODE);
            stateInit = 4;
        break;
        case 4:   // default movement distance
            mainMbSendCmd(&mbCmdPosLeft);
            HAL_Delay(20);
            mainMbSendCmd(&mbCmdPosRight);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED_POSITION_MODE);
            HAL_Delay(100);
            stateInit = 5;
        break;
        case 5:   // position mode initialize complete
          STM_EVAL_LEDOn(LED_POSITION_MODE);
          flagEnableDisable = 1;
          stateInit = 0;
          initializing = 0;
        break;

        /* speed mode initialization  */
        case 10:    // set speed mode
            mainMbSendCmd(&mbCmdSpeedMode);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED_SPEED_MODE);
            stateInit = 11;
        break;
        case 11:    // stop motor
            mainMbSendCmd(&mbCmdSpeedStop);
            HAL_Delay(100);
            STM_EVAL_LEDOn(LED_SPEED_MODE);
            stateInit = 12;
        break;
        case 12:    // set speed,acc,dec
            mainMbSendCmd(&mbCmdSpeedAccDecLeft);
            HAL_Delay(20);
            mainMbSendCmd(&mbCmdSpeedAccDecRight);
            HAL_Delay(100);
            STM_EVAL_LEDOff(LED_SPEED_MODE);
            HAL_Delay(100);
            stateInit = 13;
        break;
        case 13:    // speed mode initialize complete
          STM_EVAL_LEDOn(LED_SPEED_MODE);
          flagEnableDisable = 1;
          stateInit = 0;
          initializing = 0;
        break;       
        default:

        break;
      }
      
      SYSTICK_TIMER20 = 0;
    }
  }
}
/* Position mode movement control when  press ps2 joystick button */
void mainPosModeCtrl(void)
{
  uint16_t usCRC16 = 0x0000;
  
  switch(curKeyValue)
  {
    case PSB_UP:                    // forward
      if(curKeyState == PS2_KEY_SHORT_MASK)
      {
        mainMbSendCmd(&mbCmdPosStart);
        HAL_Delay(20);
        mainMbSendCmd(&mbCmdInitial);
        HAL_Delay(20); 
        curPositionLeft += 0x0F00;
        curPositionRight += 0x0F00;
        flagLeftLegRun = LAST_MOVE_FORWARD;
        flagRightLegRun = LAST_MOVE_FORWARD;
      }
    break;
    case PSB_DOWN:                  // backward,this mechanic dog can't move backward

    break;
    case PSB_RIGHT:                 // Turn right
      //if(curKeyState == PS2_KEY_HOLD_MASK)
      {
        mainMbSendCmd(&mbCmdPosStopRight);
        HAL_Delay(20);
        mainMbSendCmd(&mbCmdPosStartLeft);
        HAL_Delay(20);
        mainMbSendCmd(&mbCmdInitial);
        HAL_Delay(100);  
        curPositionLeft += 0x0F00;       
        
      }
    break;
    case PSB_LEFT:                  // Turn left
      //if(curKeyState == PS2_KEY_HOLD_MASK)
      {
        mainMbSendCmd(&mbCmdPosStopLeft);
        HAL_Delay(20);
        mainMbSendCmd(&mbCmdPosStartRight);
        HAL_Delay(20);
        mainMbSendCmd(&mbCmdInitial);
        HAL_Delay(100);  
        curPositionRight += 0x0F00;       
        
      }
    break;
    case PSB_TRIANGLE:              // acceleration

    break;
    case PSB_CROSS:                 // Decelleration    

    break;
    case PSB_SQUARE:                // Stop
      mainMbSendCmd(&mbCmdPosStop);
      flagLeftLegRun = LAST_MOVE_STOP;
      flagRightLegRun = LAST_MOVE_STOP;
      HAL_Delay(10);
    break;
    case PSB_CIRCLE:                // Home

    break;
    default:

    break;
  }

}
/* Speed mode movement control when  press ps2 joystick button */
void mainSpeedModeCtrl(void)
{
  uint16_t usCRC16 = 0x0000;
  
  switch(curKeyValue)
  {
    case PSB_UP:                    // forward
      if(flagEnableDisable == 0)
      {
        break;
      }
      if(curKeyState == PS2_KEY_SHORT_MASK)
      {
        curLegSpeedLeft = (mbCmdSpeedLeft.cmd[9] << 8) | mbCmdSpeedLeft.cmd[10];
        curLegSpeedRight = (mbCmdSpeedRight.cmd[9] << 8) | mbCmdSpeedRight.cmd[10];
        if(curLegSpeedRight > (65536-curLegSpeedLeft))
        {
          curLegSpeedRight = 65536-curLegSpeedLeft;
          mbCmdSpeedRight.cmd[9] = curLegSpeedRight >> 8;
          mbCmdSpeedRight.cmd[10] = curLegSpeedRight & 0x00FF;
          usCRC16 = usMBCRC16(&(mbCmdSpeedRight.cmd[0]),mbCmdSpeedRight.len-2);
          mbCmdSpeedRight.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
          mbCmdSpeedRight.cmd[12] = (uint8_t)(usCRC16 >> 8);
          mainMbSendCmd(&mbCmdSpeedRight);
          HAL_Delay(0);
        }
        else if(curLegSpeedRight < (65536-curLegSpeedLeft))
        {
          curLegSpeedLeft = 65536-curLegSpeedRight;
          mbCmdSpeedLeft.cmd[9] = curLegSpeedLeft >> 8;
          mbCmdSpeedLeft.cmd[10] = curLegSpeedLeft & 0x00FF;
          usCRC16 = usMBCRC16(&(mbCmdSpeedLeft.cmd[0]),mbCmdSpeedLeft.len-2);
          mbCmdSpeedLeft.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
          mbCmdSpeedLeft.cmd[12] = (uint8_t)(usCRC16 >> 8);
          mainMbSendCmd(&mbCmdSpeedLeft);
          HAL_Delay(0);
        }
        // avoid send two command at same time which will cause motor driver lost connection
        if((flagLeftLegRun == LAST_MOVE_STOP) && (flagRightLegRun == LAST_MOVE_STOP))
        {
          mainMbSendCmd(&mbCmdSpeedStart);
          flagLeftLegRun = LAST_MOVE_FORWARD;
          flagRightLegRun = LAST_MOVE_FORWARD;
        }
        else
        {
          if(flagLeftLegRun == LAST_MOVE_STOP)
          {
            mainMbSendCmd(&mbCmdSpeedStartLeft);
            flagLeftLegRun = LAST_MOVE_FORWARD;
          }
          if(flagRightLegRun == LAST_MOVE_STOP)
          {
            mainMbSendCmd(&mbCmdSpeedStartRight);
            flagRightLegRun = LAST_MOVE_FORWARD;
          } 
        }
      }
    break;
    case PSB_DOWN:                  // backward,this mechanic dog can't move backward

    break;
    case PSB_RIGHT:                 // Turn right
      if( (flagEnableDisable == 1) && (curKeyState == PS2_KEY_SHORT_MASK) )
      {
        curLegSpeedLeft = (mbCmdSpeedLeft.cmd[9] << 8) | mbCmdSpeedLeft.cmd[10];
        if(curLegSpeedLeft != 250)
        {
          curLegSpeedLeft = 250;
          mbCmdSpeedLeft.cmd[9] = curLegSpeedLeft >> 8;
          mbCmdSpeedLeft.cmd[10] = curLegSpeedLeft & 0x00FF;
          usCRC16 = usMBCRC16(&(mbCmdSpeedLeft.cmd[0]),mbCmdSpeedLeft.len-2);
          mbCmdSpeedLeft.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
          mbCmdSpeedLeft.cmd[12] = (uint8_t)(usCRC16 >> 8);
          mainMbSendCmd(&mbCmdSpeedLeft);
        }
        
        curLegSpeedRight = (mbCmdSpeedRight.cmd[9] << 8) | mbCmdSpeedRight.cmd[10];
        if(curLegSpeedRight != 100)
        {
          curLegSpeedRight = 100;
          mbCmdSpeedRight.cmd[9] = curLegSpeedRight >> 8;
          mbCmdSpeedRight.cmd[10] = curLegSpeedRight & 0x00FF;
          usCRC16 = usMBCRC16(&(mbCmdSpeedRight.cmd[0]),mbCmdSpeedRight.len-2);
          mbCmdSpeedRight.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
          mbCmdSpeedRight.cmd[12] = (uint8_t)(usCRC16 >> 8);
          mainMbSendCmd(&mbCmdSpeedRight);
        }
        
        if((flagLeftLegRun == LAST_MOVE_STOP) || (flagRightLegRun == LAST_MOVE_STOP))
        {
          mainMbSendCmd(&mbCmdSpeedStart);
          flagLeftLegRun = LAST_MOVE_FORWARD;
          flagRightLegRun = LAST_MOVE_FORWARD;
        }
      }

    break;
    case PSB_LEFT:                  // Turn left
      if( (flagEnableDisable == 1) && (curKeyState == PS2_KEY_SHORT_MASK) )
      {
        curLegSpeedLeft = (mbCmdSpeedLeft.cmd[9] << 8) | mbCmdSpeedLeft.cmd[10];
        if(curLegSpeedLeft != 100)
        {
          curLegSpeedLeft = 100;
          mbCmdSpeedLeft.cmd[9] = curLegSpeedLeft >> 8;
          mbCmdSpeedLeft.cmd[10] = curLegSpeedLeft & 0x00FF;
          usCRC16 = usMBCRC16(&(mbCmdSpeedLeft.cmd[0]),mbCmdSpeedLeft.len-2);
          mbCmdSpeedLeft.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
          mbCmdSpeedLeft.cmd[12] = (uint8_t)(usCRC16 >> 8);
          mainMbSendCmd(&mbCmdSpeedLeft);
        }
        
        curLegSpeedRight = (mbCmdSpeedRight.cmd[9] << 8) | mbCmdSpeedRight.cmd[10];
        if(curLegSpeedRight != 250)
        {
          curLegSpeedRight = 250;
          mbCmdSpeedRight.cmd[9] = curLegSpeedRight >> 8;
          mbCmdSpeedRight.cmd[10] = curLegSpeedRight & 0x00FF;
          usCRC16 = usMBCRC16(&(mbCmdSpeedRight.cmd[0]),mbCmdSpeedRight.len-2);
          mbCmdSpeedRight.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
          mbCmdSpeedRight.cmd[12] = (uint8_t)(usCRC16 >> 8);
          mainMbSendCmd(&mbCmdSpeedRight);
        }
        
        if((flagLeftLegRun == LAST_MOVE_STOP) || (flagRightLegRun == LAST_MOVE_STOP))
        {
          mainMbSendCmd(&mbCmdSpeedStart);
          flagLeftLegRun = LAST_MOVE_FORWARD;
          flagRightLegRun = LAST_MOVE_FORWARD;
        }
      }

    break;
    case PSB_TRIANGLE:              // acceleration
      if( (flagEnableDisable == 1) && (curKeyState == PS2_KEY_SHORT_MASK) )
      {
         //curKeyState = 0;
         curLegSpeedLeft = (mbCmdSpeedLeft.cmd[9] << 8) | mbCmdSpeedLeft.cmd[10];
         if(curLegSpeedLeft > MIN_MOTOR_SPEED)
         {
            curLegSpeedLeft -= 50;
            mbCmdSpeedLeft.cmd[9] = curLegSpeedLeft >> 8;
            mbCmdSpeedLeft.cmd[10] = curLegSpeedLeft & 0x00FF;
            usCRC16 = usMBCRC16(&(mbCmdSpeedLeft.cmd[0]),mbCmdSpeedLeft.len-2);
            mbCmdSpeedLeft.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
            mbCmdSpeedLeft.cmd[12] = (uint8_t)(usCRC16 >> 8);
            if(flagLeftLegRun == LAST_MOVE_FORWARD)
            {
              mainMbSendCmd(&mbCmdSpeedLeft);
            }
         }

         curLegSpeedRight = (mbCmdSpeedRight.cmd[9] << 8) | mbCmdSpeedRight.cmd[10];
         if(curLegSpeedRight < MAX_MOTOR_SPEED)
         {
            curLegSpeedRight += 50;
            mbCmdSpeedRight.cmd[9] = curLegSpeedRight >> 8;
            mbCmdSpeedRight.cmd[10] = curLegSpeedRight & 0x00FF;
            usCRC16 = usMBCRC16(&(mbCmdSpeedRight.cmd[0]),mbCmdSpeedRight.len-2);
            mbCmdSpeedRight.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
            mbCmdSpeedRight.cmd[12] = (uint8_t)(usCRC16 >> 8);
            if(flagRightLegRun == LAST_MOVE_FORWARD)
            {
              mainMbSendCmd(&mbCmdSpeedRight);          
            }
         }
       }
    break;
    case PSB_CROSS:                 // Decelleration    
      if( (flagEnableDisable == 1) && (curKeyState == PS2_KEY_SHORT_MASK) )
      {
         //curKeyState = 0;
         curLegSpeedLeft = (mbCmdSpeedLeft.cmd[9] << 8) | mbCmdSpeedLeft.cmd[10];
         if(curLegSpeedLeft  < 65435)
         {
            curLegSpeedLeft += 50;
            mbCmdSpeedLeft.cmd[9] = curLegSpeedLeft >> 8;
            mbCmdSpeedLeft.cmd[10] = curLegSpeedLeft & 0x00FF;
            usCRC16 = usMBCRC16(&(mbCmdSpeedLeft.cmd[0]),mbCmdSpeedLeft.len-2);
            mbCmdSpeedLeft.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
            mbCmdSpeedLeft.cmd[12] = (uint8_t)(usCRC16 >> 8);
            if(flagLeftLegRun == LAST_MOVE_FORWARD)
            {
              mainMbSendCmd(&mbCmdSpeedLeft);
            }
         }

         curLegSpeedRight = (mbCmdSpeedRight.cmd[9] << 8) | mbCmdSpeedRight.cmd[10];
         if(curLegSpeedRight > 101)
         {
            curLegSpeedRight -= 50;
            mbCmdSpeedRight.cmd[9] = curLegSpeedRight >> 8;
            mbCmdSpeedRight.cmd[10] = curLegSpeedRight & 0x00FF;
            usCRC16 = usMBCRC16(&(mbCmdSpeedRight.cmd[0]),mbCmdSpeedRight.len-2);
            mbCmdSpeedRight.cmd[11] = (uint8_t)(usCRC16 & 0xFF);
            mbCmdSpeedRight.cmd[12] = (uint8_t)(usCRC16 >> 8);
            if(flagRightLegRun == LAST_MOVE_FORWARD)
            {
              mainMbSendCmd(&mbCmdSpeedRight);          
            }
         }
       }
    break;
    case PSB_SQUARE:                // Stop
      if(flagEnableDisable == 0)
      {
        break;
      }
      if((flagLeftLegRun == LAST_MOVE_FORWARD) && (flagRightLegRun == LAST_MOVE_FORWARD))
      {
        mainMbSendCmd(&mbCmdSpeedStop);
        flagLeftLegRun = LAST_MOVE_STOP;
        flagRightLegRun = LAST_MOVE_STOP;
      }
      else
      {
        if(flagLeftLegRun == LAST_MOVE_FORWARD)
        {
          mainMbSendCmd(&mbCmdSpeedStopLeft);
          flagLeftLegRun = LAST_MOVE_STOP;
        }
        if(flagRightLegRun == LAST_MOVE_FORWARD)
        {
          mainMbSendCmd(&mbCmdSpeedStopRight);
          flagRightLegRun = LAST_MOVE_STOP;
        } 
      }

    break;
    case PSB_CIRCLE:                // Disable
      if((flagLeftLegRun == LAST_MOVE_FORWARD) && (flagRightLegRun == LAST_MOVE_FORWARD))
      {
        mainMbSendCmd(&mbCmdSpeedStop);
        flagLeftLegRun = LAST_MOVE_STOP;
        flagRightLegRun = LAST_MOVE_STOP;
      }
      else
      {
        if(flagLeftLegRun == LAST_MOVE_FORWARD)
        {
          mainMbSendCmd(&mbCmdSpeedStopLeft);
          flagLeftLegRun = LAST_MOVE_STOP;
        }
        if(flagRightLegRun == LAST_MOVE_FORWARD)
        {
          mainMbSendCmd(&mbCmdSpeedStopRight);
          flagRightLegRun = LAST_MOVE_STOP;
        } 
      }
      HAL_Delay(2);
      mainMbSendCmd(&mbCmdDisable);
      flagEnableDisable = 0;
    break;
    case PSB_START:                // Enable
      if(flagEnableDisable == 0)
      {
        stateInit = 0;
        initializing = 1;
        mainAutoInit();
      }
    break;
    default:

    break;
  }

}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    int i = 0;
    uint16_t usCRC16 = 0x0000;


    int waitChoseMoveMode = 1;


  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */ 
  HAL_Init();
 
  /*  PS2 joystick remote controller initialize  ----*/
  PS2_Init();			   

  /* Tamper Button Configuration -------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDOff(LED4);
  
  /* USART configuration and initialize ----------*/
  USART_Config();
  /* Enable the Rx buffer not empty interrupt */
  USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
  USART_ClearITPendingBit(USARTx, USART_IT_TC);
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); 
  HAL_Delay(100);
  STM_EVAL_LEDOn(LED4);   

  /* Wait until Tamper Button is pressed */
  //while (STM_EVAL_PBGetState(BUTTON_USER)==RESET);  
  while(waitChoseMoveMode)
  {
    if(1 == SYSTICK_TIMER20)        
    {
      curKey = PS2_KeyScan();
      curKeyValue = curKey & PS2_KEY_VALUE_MASK;
      if(curKeyValue == PSB_L1)      // speed mode  
      {
        moveMode = MOVE_MODE_SPEED;
        waitChoseMoveMode = 0;
        STM_EVAL_LEDOff(LED4);
      }
      else if(curKeyValue == PSB_R1)        // position mode
      {    
        moveMode = MOVE_MODE_POSITION;
        waitChoseMoveMode = 0;
        STM_EVAL_LEDOn(LED3); 
      }  
      SYSTICK_TIMER20 = 0;      
    }
  }
  HAL_Delay(100);
  while(PS2_GetKey() != PSB_START);
  STM_EVAL_LEDOn(LED4);
  HAL_Delay(100);

  mainAutoInit();
  
  while(1)
  {
     /*******************      10systick(100ms) scan cycle task ***********/
      if(1 == SYSTICK_TIMER10)        
      {
        curKey = PS2_KeyScan();
        curKeyValue = curKey & PS2_KEY_VALUE_MASK;
        curKeyState = curKey & PS2_KEY_STATE_MASK;
        lastKeyValue = curKeyValue;
        if(curKey == PSB_NONE)        // stop when lost connection/no button pressed of PS2 joystick
        {
          if(ps2KeyTimeout++ > MAX_PS2_TIMEOUT)
          {
            ps2KeyTimeout = 0;
            if((flagLeftLegRun == LAST_MOVE_FORWARD) || (flagRightLegRun == LAST_MOVE_FORWARD))
            {
              if(moveMode == MOVE_MODE_POSITION)
              {
                mainMbSendCmd(&mbCmdPosStop);
              }
              else if(moveMode == MOVE_MODE_SPEED)
              {
                mainMbSendCmd(&mbCmdSpeedStop);
              }
              
              HAL_Delay(10);
              flagLeftLegRun = LAST_MOVE_STOP;
              flagRightLegRun = LAST_MOVE_STOP;
            }
          }
        }
        else
        {
          #if(USE_PRINTF_DEBUG)
            printf("  \n   %d  is  pressed.  \n",curKey);
          #endif
          ps2KeyTimeout = 0;
          if(oldKeyValue != curKeyValue)
            mbCmdDelay = 0;
          if(moveMode == MOVE_MODE_POSITION)
          {
            mainPosModeCtrl();
          }
          else if(moveMode == MOVE_MODE_SPEED)
          {
            mainSpeedModeCtrl();
          }

          oldKeyValue = curKeyValue; 
 
      }

      curKeyValue = PSB_NONE;
      curKeyState = 0;  
      SYSTICK_TIMER10 = 0;
    }

   /**********************  None cycle task  *******************/
  
  
    
        
  }

}


/*---------------------------------------------------------------------------*/


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
