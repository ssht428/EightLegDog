/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File：PS2驱动程序
Author：pinggai    Version:1.0     Data:2015/05/16
Description: PS2驱动程序
**** 2020.10.22: V1.1: use spi to drive ps2 joystick,max frequency<312.5khz
**********************************************************/	 

#include "ps2_spi.h"

uint16_t ps2Key;	  
uint8_t ps2Cmd[9] = {0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t ps2Data[9] = {0};
uint8_t sendCmd = 1;
uint8_t ps2 = 0;
uint16_t ps2KeyMASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_UP,
    PSB_RIGHT,
    PSB_DOWN,
    PSB_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	
uint16_t spiDelay = 18;
  

void delayus(uint32_t us)
{
	__IO uint32_t i = 0;
	us *= 5;	
	while(us--)
		i++;
}

/**
  * @brief  Initializes SPI1 for PS2 joystick
  * PS2 connect to SPI4, VCC->3V,
  *  DI/DAT <-> PE5(SPI4_MISO)    , DO/CMD <-> PE6(SPI4_MOSI) , 
  *  CS <-> PE4(SPI4_NSS),  CLK <-> PE2(SPI4_SCK) 
  * @param  None
  * @retval None
  */
static void PS2_SPI_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph and CS(PE4),SCK(PE2), MISO(PE5) and MOSI(PE6) GPIO clocks */
  PS2_SPI_CLK_ENABLE();   
  PS2_SPI_GPIO_CLK_ENABLE();
  PS2_CS_CLK_ENABLE();

  GPIO_PinAFConfig(PS2_SPI_GPIO_PORT, PS2_SPI_SCK_PINSOURCE, PS2_SPI_AF);
  GPIO_PinAFConfig(PS2_SPI_GPIO_PORT, PS2_SPI_MISO_PINSOURCE, PS2_SPI_AF);
  GPIO_PinAFConfig(PS2_SPI_GPIO_PORT, PS2_SPI_MOSI_PINSOURCE, PS2_SPI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = PS2_SPI_SCK_PIN;
  GPIO_Init(PS2_SPI_GPIO_PORT, &GPIO_InitStructure);
  GPIO_SetBits(PS2_SPI_GPIO_PORT, PS2_SPI_SCK_PIN);
  
  /* SPI  MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin =  PS2_SPI_MISO_PIN;
  GPIO_Init(PS2_SPI_GPIO_PORT, &GPIO_InitStructure);
  GPIO_SetBits(PS2_SPI_GPIO_PORT, PS2_SPI_MISO_PIN);

  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin = PS2_SPI_MOSI_PIN;
  GPIO_Init(PS2_SPI_GPIO_PORT, &GPIO_InitStructure);
  GPIO_SetBits(PS2_SPI_GPIO_PORT, PS2_SPI_MOSI_PIN);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(PS2_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(PS2_SPI, &SPI_InitStructure);

  /* Enable PS2_SPI  */
  SPI_Cmd(PS2_SPI, ENABLE);
  
  /* Configure GPIO PIN for PS2 Chip select */
  GPIO_InitStructure.GPIO_Pin = PS2_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(PS2_SPI_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(PS2_SPI_GPIO_PORT, PS2_CS_PIN);
  
}  

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t PS2_SPI_SendByte(uint8_t byte)
{
  __IO uint32_t timeout = 160000;
  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
    
  }
  
  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(PS2_SPI, (uint16_t)byte);
  /* Wait to receive a Byte */
  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if(0 == (timeout--))
      break;
  }
  delayus(20);
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(PS2_SPI);
}

void PS2_ShortPoll0(void)
{
  uint16_t  i=0;
  /* Set chip select Low at the start of the transmission */
  PS2_SPI_CS_RESET();
  delayus(spiDelay);
  ps2Cmd[0] = 0x03;
  ps2Cmd[1] = 0x84;
  ps2Cmd[2] = 0x00;
  ps2Cmd[3] = 0x00;
  ps2Cmd[4] = 0x00;
    
  for(i=0;i<5;i++)
  {
    ps2Data[i] = PS2_SPI_SendByte(ps2Cmd[i]);
  }

  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {

  }
  delayus(spiDelay);
  /* Set chip select High at the end of the transmission */ 
  PS2_SPI_CS_SET(); 
  delayus(spiDelay);
    
}


void PS2_ShortPoll1(void)
{
  uint16_t  i=0;
  /* Set chip select Low at the start of the transmission */
  PS2_SPI_CS_RESET();
  delayus(spiDelay);
  ps2Cmd[0] = 0x01;
  ps2Cmd[1] = 0x42;
  ps2Cmd[2] = 0x00;
  ps2Cmd[3] = 0x00;
  ps2Cmd[4] = 0x00;
  
  for(i=0;i<5;i++)
  {
    ps2Data[i] = PS2_SPI_SendByte(ps2Cmd[i]);
  }

  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {

  }
  delayus(spiDelay);
  /* Set chip select High at the end of the transmission */ 
  PS2_SPI_CS_SET(); 
  delayus(spiDelay);
  
}
void PS2_EnterConfig(void)
{
  uint16_t  i=0;
  /* Set chip select Low at the start of the transmission */
  PS2_SPI_CS_RESET();
  delayus(spiDelay);
  ps2Cmd[0] = 0x01;
  ps2Cmd[1] = 0x43;
  ps2Cmd[2] = 0x00;
  ps2Cmd[3] = 0x01;
  ps2Cmd[4] = 0x00;
  ps2Cmd[5] = 0x00;
  ps2Cmd[6] = 0x00;
  ps2Cmd[7] = 0x00;
  ps2Cmd[8] = 0x00;
  
  for(i=0;i<9;i++)
  {
    ps2Data[i] = PS2_SPI_SendByte(ps2Cmd[i]);
  }

  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {

  }
  delayus(spiDelay);
  /* Set chip select High at the end of the transmission */ 
  PS2_SPI_CS_SET(); 
  delayus(spiDelay);
  
}
void PS2_ExitConfig(void)
{
  uint16_t  i=0;
  /* Set chip select Low at the start of the transmission */
  PS2_SPI_CS_RESET();
  delayus(spiDelay);
  ps2Cmd[0] = 0x01;
  ps2Cmd[1] = 0x43;
  ps2Cmd[2] = 0x00;
  ps2Cmd[3] = 0x00;   
  ps2Cmd[4] = 0x5A;   
  ps2Cmd[5] = 0x5A;
  ps2Cmd[6] = 0x5A;
  ps2Cmd[7] = 0x5A;
  ps2Cmd[8] = 0x5A;
  
  for(i=0;i<9;i++)
  {
    ps2Data[i] = PS2_SPI_SendByte(ps2Cmd[i]);
  }

  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {

  }
  delayus(spiDelay);
  /* Set chip select High at the end of the transmission */ 
  PS2_SPI_CS_SET(); 
  delayus(spiDelay);
  
}
void PS2_TurnOnAnalogMode(void)
{
  uint16_t  i=0;
  /* Set chip select Low at the start of the transmission */
  PS2_SPI_CS_RESET();
  delayus(spiDelay);
  ps2Cmd[0] = 0x01;
  ps2Cmd[1] = 0x44;
  ps2Cmd[2] = 0x00;
  ps2Cmd[3] = 0x01;   // analog=0x01; digital=0x00, 
  ps2Cmd[4] = 0xEE;   // 0x33:lock MODE,0xEE: no lock,it can change mode through button
  ps2Cmd[5] = 0x00;
  ps2Cmd[6] = 0x00;
  ps2Cmd[7] = 0x00;
  ps2Cmd[8] = 0x00;
  
  for(i=0;i<9;i++)
  {
    ps2Data[i] = PS2_SPI_SendByte(ps2Cmd[i]);
  }

  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {

  }
  delayus(spiDelay);
  /* Set chip select High at the end of the transmission */ 
  PS2_SPI_CS_SET(); 
  delayus(spiDelay);
  
}

uint16_t PS2_GetKey(void)
{
  uint16_t  i=0;
  uint16_t  curKey = 0xFFFF;    
  
  /* Set chip select Low at the start of the transmission */
  PS2_SPI_CS_RESET();
  delayus(spiDelay);
  ps2Cmd[0] = 0x01;
  ps2Cmd[1] = 0x42;
  ps2Cmd[2] = 0x00;
  ps2Cmd[3] = 0x00;
  ps2Cmd[4] = 0x00;
  ps2Cmd[5] = 0x00;
  ps2Cmd[6] = 0x00;
  ps2Cmd[7] = 0x00;
  ps2Cmd[8] = 0x00;
  
  for(i=0;i<9;i++)
  {
    ps2Data[i] = PS2_SPI_SendByte(ps2Cmd[i]);
  }

  while (SPI_I2S_GetFlagStatus(PS2_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {

  }
  delayus(spiDelay);
  /* Set chip select High at the end of the transmission */ 
  PS2_SPI_CS_SET(); 
  delayus(spiDelay);
 

	curKey=(ps2Data[3]<<8)|ps2Data[4];     
	return curKey;        

}

/*----------------------------------------------------------
    Name       : unsigned int PS2_KeyScan(void)
    Description:  	* put this function in cyclic program
							* Convert the sample into a key number.
							* If the key is different than the last key seen, 
							* return true to indicate the change, otherwise return false.
							* Always returns the current value in reference parameter k.
							* This allows clients to query the present value,
							* or ignore the value when it is unchanging.
    Parameter  : None
    Returns    :  key value
  ----------------------------------------------------------*/
unsigned int PS2_KeyScan(void)
{
	unsigned int i;
	unsigned int key,curKey;		 
  static unsigned int keyLastTick = 0;		// record the key last period for debounce,unit=systick
	static unsigned short lastKey = PSB_NONE;  
	static unsigned int keyState = 0;
	
	curKey = PSB_NONE;
  key = PS2_GetKey();
  
	/*  Check to see if current key number is different than last */
	switch(keyState)
	{
		case 0:
			if(key != PSB_NONE)
			{
				keyState = 1;
				lastKey = key;
			}
    break;
		case 1:			//wait for debounce
			if((key != lastKey) || (key == PSB_NONE))
			{
				lastKey = PSB_NONE; 						// Update value in last_key
				keyState = 0;
				keyLastTick = 0;
			}
			else
			{
				if((keyLastTick++) >= PS2_DEBOUNCE_TICK)
				{
					//curKey = lastKey | PS2_KEY_SHORT_MASK;
					keyState = 2;
				}
			}
		break;
		case 2:			// check for short or long press
			if((key != lastKey) || (key == PSB_NONE))
			{
        curKey = lastKey | PS2_KEY_SHORT_MASK;
				lastKey = PSB_NONE; 						
				keyState = 0;
				keyLastTick = 0;	
			}
			else
			{
        /*
				if((keyLastTick++) > PS2_LONG_TICK)
				{
					curKey = key | PS2_KEY_LONG_MASK;
					keyState = 3;
					keyLastTick = 0;	
				}
				*/
        
        keyLastTick++;
        if((keyLastTick) >= PS2_HOLD_TICK)
				{
					curKey = key | PS2_KEY_HOLD_MASK;
					keyState = 3;
					keyLastTick = 0;	
				}     
				else if((keyLastTick) >= PS2_LONG_TICK)
				{
					curKey = key | PS2_KEY_LONG_MASK;
					keyState = 3;
					keyLastTick = 0;	
				}
        else if((keyLastTick % (PS2_DEBOUNCE_TICK*2)) == 0)
        {
          //curKey = key | PS2_KEY_SHORT_MASK;
        }
        
			}				
		break;
		case 3:			// wait for release key			
			if((key != lastKey) || (key == PSB_NONE))
			{
        curKey = lastKey | PS2_KEY_LONG_MASK;
				lastKey = PSB_NONE; 						
				keyState = 0;
				curKey = PSB_NONE;
			}				
			else
				curKey = key | PS2_KEY_HOLD_MASK;
		break;
		
		default:			
			
		break;

	}

  return curKey;
}

void PS2_Init(void)
{
  PS2_SPI_Init();
  delayus(500);
	PS2_ShortPoll0();
	PS2_ShortPoll1();
	PS2_ShortPoll1();
	PS2_EnterConfig();		
	//PS2_TurnOnAnalogMode();		
	PS2_ExitConfig();		
}




