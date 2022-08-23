#include "timer.h"
//定时器初始化
//提供通用定时器2的初始化配一个定时中断
//使用时主函数里添加头文件
//加入函数Timer_TIM2_Init(,);
//高级定时器和普通的有所不同
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitSture;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseInitStrue.TIM_Period=arr;//自动装载值
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//暂时不用
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStrue);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//使能定时器更新中断
	
	NVIC_InitSture.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitSture.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitSture.NVIC_IRQChannelSubPriority=3;
	NVIC_InitSture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitSture);
	
	TIM_Cmd(TIM3,ENABLE);
}





