#ifndef __ENCODER_H
#define __ENCODER_H	 
#include "sys.h"

void Encoder_TIM2_Init(void);
void Encoder_TIM4_Init(void);
int Read_Speed(int TIMx);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
int Read_Speed(int TIMx);
int Mind_Read_Speed(int TIMx);
void Encoder_Init_TIM3(void);


void Encoder_Init_TIM_Exit0(void);//用于检测编码器A相使用PA0引脚
void Encoder_Init_TIM_Exit1(void);//用于检测编码器B相使用PA1引脚
void EXTI0_IRQHandler(void);  //外部中断线0服务函数	
void EXTI1_IRQHandler(void);//外部中断线1服务函数
void EXTI6_IRQHandler(void);
void EXTI7_IRQHandler(void);
int Read_Encoder(int TIMX);
void TIM4_Int_Init(u16 arr,u16 psc);
void Encoder_Init_TIM_Exit6(void);
void Encoder_Init_TIM_Exit7(void);
void TIM2_Int_Init(u16 arr,u16 psc);

#endif
