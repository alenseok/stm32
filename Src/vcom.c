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
  
#include "vcom.h"
//#include "stm32f1xx_hal.h"
#include <stdarg.h>


//#define BUFSIZE	1024+16
#define BUFSIZE	1024+2


static uint16_t iw;
static char buff[BUFSIZE];

#if 1
uint8_t U1_Buff[BUFSIZE] = {0, };	// al : for UART RX interrupt of UART1
int U1_Head = 0;					// al : for UART RX interrupt of UART1
int U1_Tail = 0;					// al : for UART RX interrupt of UART1

uint8_t U2_Buff[BUFSIZE] = {0, };	// al : for UART RX interrupt of UART2
int U2_Head = 0;					// al : for UART RX interrupt of UART2
int U2_Tail = 0;					// al : for UART RX interrupt of UART2
#endif


//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;

/*
	for printf(...)
*/
PUTCHAR_PROTOTYPE
{
/*	Place yours implementation of fputc here	*/
/*	e.g. vrite a character to the USART1 and loop until the end of transmission	*/
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
	//HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);

	if(ch == '\n')
	{
		ch = '\r';
		HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
	}
	return ch;
}

/*
	for PRINTF(...)
*/
void vcom_Send( char *format, ... )
{
	va_list args;
	va_start(args, format);

	/*convert into string at buff[0] of length iw*/
	iw = vsprintf(&buff[0], format, args);

	if(buff[iw-1] == '\n')
	{
		buff[iw] = '\r';
		iw++;
	}

	//HAL_UART_Transmit(&huart1,(uint8_t *)&buff[0], iw, 0xFFFF);
	HAL_UART_Transmit(&huart1,(uint8_t *)&buff[0], iw, 300);

	va_end(args);
}


void vcom1_Send(uint8_t *pData, uint16_t Size)
{
	uint8_t send;
	uint16_t sz;

	sz = Size;

	while(sz)
	{
		send = *(pData+(Size -sz));
		HAL_UART_Transmit(&huart1, &send, 1, 1);
		sz--;
	}
}

void vcom2_Send(uint8_t *pData, uint16_t Size)
{
	uint8_t send;
	uint16_t sz;

	sz = Size;

	while(sz)
	{
		send = *(pData+(Size -sz));
		HAL_UART_Transmit(&huart2, &send, 1, 1);
		sz--;
	}
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
	* UART TX

	char *msg;

	msg = "Before while loop\n\r";

	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);
*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


