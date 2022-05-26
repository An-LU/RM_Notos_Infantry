#include "usart4.h"
#include "stm32f4xx_dma.h"
#include "miniPC.h"
//#include "led.h"
#include "string.h"

//视觉串口4配置

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

//接收到的视觉数据暂存在这里
static uint8_t  Com4_Vision_Buffer[ VISION_BUFFER_LEN ] = {0};

static int Usart4_Clean_IDLE_Flag = 0;

static DMA_InitTypeDef xCom4DMAInit;

//视觉通信串口初始化
void UART4_Init(void)
{
	USART_InitTypeDef xUsartInit;
	GPIO_InitTypeDef xGpioInit;
	NVIC_InitTypeDef xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4, ENABLE );

	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART4 );//引脚复用
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART4 ); 

	xGpioInit.GPIO_Pin = GPIO_PIN_TX;//0
	xGpioInit.GPIO_Mode = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;//推挽复用输出
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_RX;//1
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate = 115200;   //波特率
	xUsartInit.USART_WordLength = USART_WordLength_8b;//字长8比特
	xUsartInit.USART_StopBits = USART_StopBits_1;//一个停止位
	xUsartInit.USART_Parity = USART_Parity_No;//无奇偶校验
	xUsartInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//收发模式
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制

	USART_Init( UART4, &xUsartInit );
	USART_Cmd( UART4, ENABLE );

	//使能串口空闲中断USART_IT_IDLE,连续的一串数据发送完成之后，才触发空闲中断
	USART_ITConfig( UART4, USART_IT_IDLE, ENABLE );  

	//DMA串口请求中断
	USART_DMACmd( UART4, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART4, USART_DMAReq_Tx, ENABLE );

	UART4_DMA_Init();

	//中断配置
	xNvicInit.NVIC_IRQChannel = UART4_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority = 0;//优先级
	xNvicInit.NVIC_IRQChannelSubPriority = 0;
	xNvicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &xNvicInit );		
}

//DMA配置
void UART4_DMA_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	
	DMA_DeInit( DMA1_Stream_RX );
	//while(DMA_GetCmdStatus(DMA1_Stream_RX));
	xCom4DMAInit.DMA_Channel = DMA_Channel_4;//通道选择
	
	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	xCom4DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART4->DR);//DMA外设地址
	xCom4DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Com4_Vision_Buffer;//DMA 存储器0地址，缓存数组
	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
	xCom4DMAInit.DMA_BufferSize = 100;//数据传输量 
	xCom4DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	xCom4DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	xCom4DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	xCom4DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	xCom4DMAInit.DMA_Mode = DMA_Mode_Circular;// 连续模式 
	xCom4DMAInit.DMA_Priority = DMA_Priority_VeryHigh;//最高等优先级
	xCom4DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;//不开启FIFO
	xCom4DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//FIFO 阈值选择：
	xCom4DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	xCom4DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输

	DMA_Init( DMA1_Stream_RX, &xCom4DMAInit );	
	DMA_Cmd( DMA1_Stream_RX, ENABLE);  //stream2
}

//串口4中断服务函数
void UART4_IRQHandler(void)
{	
	if(USART_GetITStatus(UART4,USART_IT_IDLE)!=RESET)//检测到空闲线路
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		Usart4_Clean_IDLE_Flag = UART4->SR ;
		Usart4_Clean_IDLE_Flag = UART4->DR ;	
		
		DMA_Cmd(DMA1_Stream2,DISABLE );
		
		Usart4_Clean_IDLE_Flag = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);

		Vision_Read_Data(Com4_Vision_Buffer);//解析视觉数据	
		
		memset(Com4_Vision_Buffer, 0, 100);
		
		
		
		DMA_Cmd(DMA1_Stream2,ENABLE);//D1S2
	}
}

//串口发送函数，一次发送一个字节数据
void UART4_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART4, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART4, cData );   
}
void USRT4_SendData(uint8_t *Data)
{
	while(*Data != NULL)
	{
		UART4_SendChar(*Data++);
	}
}
