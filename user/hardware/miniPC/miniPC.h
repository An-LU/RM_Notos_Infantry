#ifndef __MINIPC_H__
#define __MINIPC_H__

/* TX */
#define    GPIO_TX                   GPIOA
#define    GPIO_PIN_TX               GPIO_Pin_0
#define    GPIO_PINSOURCE_TX         GPIO_PinSource0
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOA

/* RX */
#define    GPIO_RX                   GPIOA
#define    GPIO_PIN_RX               GPIO_Pin_1
#define    GPIO_PINSOURCE_RX         GPIO_PinSource1
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOA

/* DMA */
#define    DMA1_Stream_RX            DMA1_Stream2

/* 接收缓存 */
#define    VISION_BUFFER_LEN           100   //稍微给大点

void UART4_Init(void);
void UART4_DMA_Init(void);

#endif