#include "usart.h"
#include "stm32f4xx_dma.h"
#include "miniPC.h"
#include "judge_system.h"
//#include "led.h"
#include "string.h"
#include "crc.h"

static uint8_t	Judge_Buffer[ JUDGE_BUFFER_LEN ] = { 0 };
static uint8_t  Vision_Buffer[ VISION_BUFFER_LEN ] = {0};
static int Usart7_Clean_IDLE_Flag = 0;
static int Usart8_Clean_IDLE_Flag = 0;

void Vision_Usart_DMA_Init(void)
{
	DMA_InitTypeDef DMAInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	
	DMA_DeInit( DMA1_Stream3 );
	//while(DMA_GetCmdStatus(DMA1_Stream3));
	DMAInitStruct.DMA_Channel = DMA_Channel_5;//通道选择
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	DMAInitStruct.DMA_PeripheralBaseAddr  = (uint32_t)&(UART7->DR);//DMA外设地址
	DMAInitStruct.DMA_Memory0BaseAddr     = (uint32_t)Vision_Buffer;//DMA 存储器0地址，缓存数组
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
	DMAInitStruct.DMA_BufferSize = 100;//数据传输量 
	DMAInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMAInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMAInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMAInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMAInitStruct.DMA_Mode = DMA_Mode_Circular;// 连续模式 
	DMAInitStruct.DMA_Priority = DMA_Priority_VeryHigh;//最高等优先级
	DMAInitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;//不开启FIFO
	DMAInitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//FIFO 阈值选择：
	DMAInitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMAInitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输

	DMA_Init( DMA1_Stream3, &DMAInitStruct );	
	DMA_Cmd( DMA1_Stream3, ENABLE);
}
void Vision_Usart_Init()
{
	USART_InitTypeDef UsartInitStruct;
	GPIO_InitTypeDef GPIOInitStruct;
	NVIC_InitTypeDef NvicInitStruct;
	/* 使能时钟 */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART7, ENABLE );
	/* 引脚复用 */
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource8, GPIO_AF_UART7 );
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource7, GPIO_AF_UART7 ); 
	/*	GPIO TxRx */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOInitStruct.GPIO_OType = GPIO_OType_PP;//推挽复用输出
	GPIOInitStruct.GPIO_Pin = Vision_Uart_Tx | Vision_Uart_Rx;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( Vision_Uart_GPIO, &GPIOInitStruct );
	/* 串口配置 */
	UsartInitStruct.USART_BaudRate = 115200;   //波特率
	UsartInitStruct.USART_WordLength = USART_WordLength_8b;//字长8比特
	UsartInitStruct.USART_StopBits = USART_StopBits_1;//一个停止位
	UsartInitStruct.USART_Parity = USART_Parity_No;//无奇偶校验
	UsartInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//收发模式
	UsartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制

	USART_Init( Vision_Uart, &UsartInitStruct );
	USART_Cmd( Vision_Uart, ENABLE );

	//使能串口空闲中断USART_IT_IDLE,连续的一串数据发送完成之后，才触发空闲中断
	USART_ITConfig( Vision_Uart, USART_IT_IDLE, ENABLE );  

	//DMA串口请求中断
	USART_DMACmd( Vision_Uart, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( Vision_Uart, USART_DMAReq_Tx, ENABLE );

	Vision_Usart_DMA_Init();

	//中断配置
	NvicInitStruct.NVIC_IRQChannel = UART7_IRQn;
	NvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;//优先级
	NvicInitStruct.NVIC_IRQChannelSubPriority = 0;
	NvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NvicInitStruct );	
}
void Judge_Usart_DMA_Init(void)
{
	DMA_InitTypeDef DMAInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	
	DMA_DeInit( DMA1_Stream6 );
	//while(DMA_GetCmdStatus(DMA1_Stream6));
	DMAInitStruct.DMA_Channel = DMA_Channel_5;//通道选择
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	DMAInitStruct.DMA_PeripheralBaseAddr  = (uint32_t)&(UART8->DR);//DMA外设地址
	DMAInitStruct.DMA_Memory0BaseAddr     = (uint32_t)Judge_Buffer;//DMA 存储器0地址，缓存数组
	
	DMAInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
	DMAInitStruct.DMA_BufferSize = 100;//数据传输量 
	DMAInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMAInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMAInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMAInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMAInitStruct.DMA_Mode = DMA_Mode_Circular;// 连续模式 
	DMAInitStruct.DMA_Priority = DMA_Priority_VeryHigh;//最高等优先级
	DMAInitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;//不开启FIFO
	DMAInitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//FIFO 阈值选择：
	DMAInitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMAInitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输

	DMA_Init( DMA1_Stream6, &DMAInitStruct );	
	DMA_Cmd( DMA1_Stream6, ENABLE);
}
void Judge_Usart_Init(void)
{
	USART_InitTypeDef UsartInitStruct;
	GPIO_InitTypeDef GPIOInitStruct;
	NVIC_InitTypeDef NvicInitStruct;
	/* 使能时钟 */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART8, ENABLE );
	/* 引脚复用 */
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource0, GPIO_AF_UART8 );
	GPIO_PinAFConfig( Vision_Uart_GPIO, GPIO_PinSource1, GPIO_AF_UART8 ); 
	/*	GPIO TxRx */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIOInitStruct.GPIO_OType = GPIO_OType_PP;//推挽复用输出
	GPIOInitStruct.GPIO_Pin = Judge_Uart_Tx | Judge_Uart_Rx;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( Vision_Uart_GPIO, &GPIOInitStruct );
	/* 串口配置 */
	UsartInitStruct.USART_BaudRate = 115200;   //波特率
	UsartInitStruct.USART_WordLength = USART_WordLength_8b;//字长8比特
	UsartInitStruct.USART_StopBits = USART_StopBits_1;//一个停止位
	UsartInitStruct.USART_Parity = USART_Parity_No;//无奇偶校验
	UsartInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//收发模式
	UsartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制

	USART_Init( Judge_Uart, &UsartInitStruct );
	USART_Cmd( Judge_Uart, ENABLE );

	//使能串口空闲中断USART_IT_IDLE,连续的一串数据发送完成之后，才触发空闲中断
	USART_ITConfig( Judge_Uart, USART_IT_IDLE, ENABLE );  

	//DMA串口请求中断
	USART_DMACmd( Judge_Uart, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( Judge_Uart, USART_DMAReq_Tx, ENABLE );

	Judge_Usart_DMA_Init();

	//中断配置
	NvicInitStruct.NVIC_IRQChannel = UART8_IRQn;
	NvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;//优先级
	NvicInitStruct.NVIC_IRQChannelSubPriority = 0;
	NvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NvicInitStruct );	
}


//视觉串口中断服务函数
void UART7_IRQHandler(void)
{	
//	uint8_t Data[10] = {'0','1','2','3','4','5','6','7','8','9'};
	if(USART_GetITStatus(Vision_Uart,USART_IT_IDLE)!=RESET)//检测到空闲线路
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		Usart7_Clean_IDLE_Flag = UART7->SR ;
		Usart7_Clean_IDLE_Flag = UART7->DR ;	
		
		DMA_Cmd(DMA1_Stream3,DISABLE );
		
		Usart7_Clean_IDLE_Flag = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);

		Vision_Read_Data(Vision_Buffer);//解析视觉数据	
		
		memset(Vision_Buffer, 0, 100);
//		Data_Send_to_Vision();
		DMA_Cmd(DMA1_Stream3,ENABLE);
	}
}
//裁判系统串口中断服务函数
void UART8_IRQHandler(void)
{	
	if(USART_GetITStatus(Judge_Uart,USART_IT_IDLE)!=RESET)//检测到空闲线路
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		Usart8_Clean_IDLE_Flag = Judge_Uart->SR ;
		Usart8_Clean_IDLE_Flag = Judge_Uart->DR ;	
		
		DMA_Cmd(DMA1_Stream6,DISABLE );
		
		Usart8_Clean_IDLE_Flag = JUDGE_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream6);

		Judge_Read_Data(Judge_Buffer);//解析视觉数据	
		
		memset(Judge_Buffer, 0, 100);
		
		DMA_Cmd(DMA1_Stream6,ENABLE);//D1S2
	}
}
//串口发送函数，一次发送一个字节数据
void UART7_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART7, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART7, cData );   
}
void VisionUart_SendData(uint8_t *Data, uint8_t length)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		UART7_SendChar(*Data++);
		
	}
}
//串口发送函数，一次发送一个字节数据
void UART8_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART8, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART8, cData );   
}
void JudgeUart_SendData(uint8_t *Data, uint8_t length)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		UART8_SendChar(*Data++);
		
	}
}

