#ifndef _USART4_H
#define _USART4_H

#include "sys.h"  



/* ���ջ��� */
#define		VISION_BUFFER_LEN           100   //��΢�����
/* �������ݰ���С */
#define		VISION_SEND_LEN		20

//extern uint8_t  Com4_Vision_Buffer[ VISION_BUFFER_LEN ];


void UART4_Init(void);
void UART4_DMA_Init(void);

void UART4_SendChar(uint8_t cData);
void USRT4_SendData(uint8_t *Data);

#endif
