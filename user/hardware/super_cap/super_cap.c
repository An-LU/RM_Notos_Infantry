#include "super_cap.h"
#include "stm32f4xx.h"

void super_cap_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //使能GPIOH时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       //GPIO输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//上拉
	GPIO_Init(GPIOH, &GPIO_InitStructure); 	//
	
	super_cap_none();
}

void super_cap_on(void){
	GPIO_SetBits(GPIOH, GPIO_Pin_12);//     超级电容放电打开
	GPIO_ResetBits(GPIOH, GPIO_Pin_11);//   超级电容充电关闭
}
void super_cap_off(void){
	GPIO_ResetBits(GPIOH, GPIO_Pin_12);//     超级电容放电关闭
	GPIO_SetBits(GPIOH, GPIO_Pin_11);//       超级电容充电打开
}
void super_cap_none(void){
	GPIO_ResetBits(GPIOH, GPIO_Pin_12);//   超级电容不工作
	GPIO_ResetBits(GPIOH, GPIO_Pin_11);//   超级电容不工作
}
