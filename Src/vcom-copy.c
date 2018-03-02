 /******************************************************************************
  * @file    vcom.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    15-September-2016
  * @brief   manages virtual com port
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
//#include "hw.h"
#include "vcom.h"
#include "stm32f1xx_hal.h"
#include <stdarg.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFSIZE 128

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t iw;
//static unsigned short int iw;
static char buff[BUFSIZE+16];
//static USART_HandleTypeDef UartHandle;
//static USART_HandleTypeDef huart2;
static UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
#if 0
void vcom_Init(void)
{
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = DBG_USART;
  
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = USART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = USART_STOPBITS_1;
  UartHandle.Init.Parity     = USART_PARITY_NONE;
  UartHandle.Init.Mode       = USART_MODE_TX_RX;
  
  if(HAL_USART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}
#endif

#if 0
void vcom_DeInit(void)
{
#if 1
  HAL_USART_DeInit(&UartHandle);
#endif
}
#endif

void vcom_Send( char *format, ... )
{
  va_list args;
  va_start(args, format);
  
  /*convert into string at buff[0] of length iw*/
  iw = vsprintf(&buff[0], format, args);
  
  //HAL_USART_Transmit(&huart2,(uint8_t *)&buff[0], iw, 300);
	HAL_UART_Transmit(&huart2,(uint8_t *)&buff[0], iw, 300);
  
  va_end(args);
}

#if 0
void vcom_Dump(uint8_t *Buffer , uint32_t Size)
{
	uint32_t Addr =0;
	uint8_t  buffer[8];
    
	
	vcom_Send("      0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F");
	while(Size--) 
	{ 
		if((Addr % 16) == 0)
		{
			sprintf((char *)buffer,"\r\n%04x:",Addr);
			vcom_Send((char *)buffer);
		}
		sprintf((char *)buffer," %02X ",Buffer[Addr]);
		vcom_Send((char *)buffer);
		Addr++;
	}
	vcom_Send("\r\n");

}
#endif