
#ifndef __STM32_USART2_H
#define __STM32_USART2_H
/******************************************************************************/
#include "stm32f4xx.h"

/******************************************************************************/
#define USART2_BUFFER_SIZE 256
/******************************************************************************/
extern char snd2_buff[USART2_BUFFER_SIZE];
extern char rcv2_buff[USART2_BUFFER_SIZE];
extern unsigned long rcv2_cntr;
extern unsigned long rcv2_flag;
/******************************************************************************/
void USART2_Init(unsigned long bound);
void USART2_SendDMA(uint32_t len);
void usart2_send_array(uint8_t *data, uint32_t len);

void uart2_init(u32 bound);
void u2_SendByte(uint8_t Byte);
void u2_SendArray(uint8_t *Array, uint16_t Length);
/******************************************************************************/


#endif


