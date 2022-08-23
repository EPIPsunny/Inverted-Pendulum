#include "encoder.h"
#include "stm32f10x.h"

extern int Encoder_A_EXTI,Encoder_B_EXTI;
extern int lspeed,rspeed;
/**************************************************************************
函数功能：外部中断采集编码器初始化
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM_Exit0(void)//用于检测编码器A相使用PA0引脚
{
   	GPIO_InitTypeDef GPIO_InitStructure;
   	EXTI_InitTypeDef EXTI_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;
   
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能PA端口时钟
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	            //端口配置
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
   	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
 	  EXTI_InitStructure.EXTI_Line=EXTI_Line0;
 	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
 	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//跳变沿触发
 	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 	  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
     
  // 配置中断优先级 													
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void Encoder_Init_TIM_Exit1(void)//用于检测编码器B相使用PA1引脚
{
   	GPIO_InitTypeDef GPIO_InitStructure;
   	EXTI_InitTypeDef EXTI_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能PA端口时钟
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	            //端口配置
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
   	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
 	EXTI_InitStructure.EXTI_Line=EXTI_Line1;
 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//跳变沿触发
 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
     
  //配置中断优先级												
   NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;	  
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

//void Encoder_Init_TIM_Exit6(void)//用于检测编码器A相使用PA0引脚
//{
//   	GPIO_InitTypeDef GPIO_InitStructure;
//   	EXTI_InitTypeDef EXTI_InitStructure;
//   	NVIC_InitTypeDef NVIC_InitStructure;
//   
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PA端口时钟
//   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	            //端口配置
//   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
//   	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
// 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
// 	  EXTI_InitStructure.EXTI_Line=EXTI_Line6;
// 	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
// 	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//跳变沿触发
// 	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// 	  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
//     
//  // 配置中断优先级 													
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

//void Encoder_Init_TIM_Exit7(void)//用于检测编码器A相使用PA0引脚
//{
//   	GPIO_InitTypeDef GPIO_InitStructure;
//   	EXTI_InitTypeDef EXTI_InitStructure;
//   	NVIC_InitTypeDef NVIC_InitStructure;
//   
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
//   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PA端口时钟
//   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	            //端口配置
//   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
//   	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
// 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);
// 	  EXTI_InitStructure.EXTI_Line=EXTI_Line7;
// 	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
// 	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//跳变沿触发
// 	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// 	  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
//     
//  // 配置中断优先级 													
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitSture;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_TimeBaseInitStrue.TIM_Period=arr;//自动装载值
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//暂时不用
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//使能定时器更新中断
	
	NVIC_InitSture.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitSture.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitSture.NVIC_IRQChannelSubPriority=2;
	NVIC_InitSture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitSture);
	
	TIM_Cmd(TIM4,ENABLE);
}

void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitSture;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStrue.TIM_Period=arr;//自动装载值
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//暂时不用
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrue);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//使能定时器更新中断
	
	NVIC_InitSture.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitSture.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitSture.NVIC_IRQChannelSubPriority=2;
	NVIC_InitSture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitSture);
	
	TIM_Cmd(TIM2,ENABLE);
}



int Read_Encoder(int TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM=(short)Encoder_A_EXTI;  Encoder_A_EXTI=0; break;	
		 case 4:  Encoder_TIM=(short)Encoder_B_EXTI;  Encoder_B_EXTI=0; break;
	   default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}

void EXTI1_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line1);
	Encoder_A_EXTI++;
}





//void EXTI0_IRQHandler(void)  //外部中断线0服务函数
//{
//		EXTI->PR=1<<0;  //清除LINE上的中断标志位
//		if(PAin(0)==0)   //这里判断检测到的是否是下降沿
//	{
//	if(PAin(1)==0)   Encoder_A_EXTI++;//看B相的电平如果是高电机就是正转则加1，否则就是反转减1
//	else             Encoder_A_EXTI--;
//	}
//	else                  //上升沿
//	{ 
//	if(PAin(1)==0)  Encoder_A_EXTI--; //B相低电平为正转，加1，高电平反转减1
//	else             Encoder_A_EXTI++;
//	}		
//}

//void EXTI1_IRQHandler(void)//外部中断线1服务函数
//{			
//		EXTI->PR=1<<1;  //清除LINE上的中断标志位
//	if(PAin(1)==1) //这里判断检测到的是否是上升沿
//	{
//	if(PAin(0)==0)  Encoder_A_EXTI++;  //看A相的电平如果是低,电机就是正转则加1，否则就是反转减1 
//	else            Encoder_A_EXTI--;
//	}
//	else
//	{
//	if(PAin(0)==0)  Encoder_A_EXTI--;
//	else            Encoder_A_EXTI++;
//	}		
//}

///////////////////////////////////////////////
void Encoder_TIM2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0 |GPIO_Pin_1;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICFilter=10;
	TIM_ICInit(TIM2,&TIM_ICInitStruct);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	TIM_SetCounter(TIM2,0);

	TIM_Cmd(TIM2,ENABLE);
}


void Encoder_TIM4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler=0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICFilter=10;
	TIM_ICInit(TIM4,&TIM_ICInitStruct);
	
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	TIM_SetCounter(TIM4,0);

	TIM_Cmd(TIM4,ENABLE);
}

/**********************
编码器
速度读取函数
入口参数：定时器
**********************/
int Read_Speed(int TIMx)
{
	int value_1;
	switch(TIMx)
	{
		case 2:value_1=(short)TIM_GetCounter(TIM2);TIM_SetCounter(TIM2,0);break;//IF是定时器2，1.采集编码器的计数值并保存。2.将定时器的计数值清零。
		case 4:value_1=(short)TIM_GetCounter(TIM4);TIM_SetCounter(TIM4,0);break;
		default:value_1=0;
	}
	return value_1;
}

int Mind_Read_Speed(int TIMx)
{
	int value_1;
	switch(TIMx)
	{
		case 2:value_1=(short)TIM_GetCounter(TIM2);break;//IF是定时器2，1.采集编码器的计数值并保存。2.将定时器的计数值清零。
		case 4:value_1=(short)TIM_GetCounter(TIM4);break;
		default:value_1=0;
	}
	return value_1;
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		//lspeed=Encoder_A_EXTI;
	}
}
	
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		//lspeed=Encoder_A_EXTI;
	}
}
///////////////////////////////////////////////////////////////////////

