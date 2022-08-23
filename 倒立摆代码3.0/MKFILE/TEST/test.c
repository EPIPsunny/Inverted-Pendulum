#include "test.h"
#include "control.h"
#include "wifi.h"
#include "delay.h"
#include "string.h"
///////////////////////////////////////////////////////////////////
float middle_check(void)//用来确定middle的值，即直立时的一个值
{
	float mid;
  mid=(float)Adc2_org;
	
	return mid;
}
///////////////////////////////////////////////////////
float rat_check(void)//用来确定rat的值
{
	float adc_up,adc_down,rat;
	USART_RX_STA1=0;
	memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
  while(USART_RX_STA1==0);
	if(USART_RX_STA1!=0)
		{					   	
			delay_ms(100);
			if (strcmp((const char *)USART_RX_BUF1, "w")==0)
			{
			  adc_up=(float)Adc2_org;
			}        
			USART_RX_STA1=0;
			memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
		}
	while(USART_RX_STA1==0);
	if(USART_RX_STA1!=0)
	{					   	
		delay_ms(100);
		if (strcmp((const char *)USART_RX_BUF1, "s")==0)
		{
		  adc_down=(float)Adc2_org;
		}        
			USART_RX_STA1=0;
			memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
	}
	rat=(adc_up-adc_down)/180;
	return rat;
}
/////////////////////////////////////////////////////////////////////////

void rat2_check(void)
{
	USART_RX_STA1=0;
	memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
  while(USART_RX_STA1==0); 
  if(USART_RX_STA1!=0)
	{					   	
		delay_ms(100);
		if (strcmp((const char *)USART_RX_BUF1, "w")==0)
		{
			TIM_SetCounter(TIM4,0);
		}        
			USART_RX_STA1=0;
			memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
	}
	while(USART_RX_STA1==0);
	if(USART_RX_STA1!=0)
	{					   	
		delay_ms(100);
		if (strcmp((const char *)USART_RX_BUF1, "s")==0)
		{
		  rat2=(float)TIM_GetCounter(TIM4)/720;
		}        
			USART_RX_STA1=0;
			memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
	}
}

void bluetooth_check(void)
{
    if(USART_RX_STA1!=0)
		{					   	
			delay_ms(100);
			if (strcmp((const char *)USART_RX_BUF1, "p0")==0)        Vertical_Kp=Vertical_Kp-10;
			if (strcmp((const char *)USART_RX_BUF1, "p1")==0)        Vertical_Kp=Vertical_Kp+10;
			if (strcmp((const char *)USART_RX_BUF1, "d0")==0)        Vertical_Kd=Vertical_Kd-10;
			if (strcmp((const char *)USART_RX_BUF1, "d1")==0)        Vertical_Kd=Vertical_Kd+10;
			if (strcmp((const char *)USART_RX_BUF1, "i1")==0) 
			{
			  Velocity_Kd=Velocity_Kd+20;
			}
			if (strcmp((const char *)USART_RX_BUF1, "i0")==0) 
			{
			  Velocity_Kd=Velocity_Kd-20;
			}
			
			if (strcmp((const char *)USART_RX_BUF1, "pp0")==0)        pos_p=pos_p-1;
			if (strcmp((const char *)USART_RX_BUF1, "pp1")==0)        pos_p=pos_p+1;
			if (strcmp((const char *)USART_RX_BUF1, "pi0")==0)        pos_i=pos_i-0.1;
			if (strcmp((const char *)USART_RX_BUF1, "pi1")==0)        pos_i=pos_i+0.1;
			if (strcmp((const char *)USART_RX_BUF1, "pd0")==0)        pos_d=pos_d-50;
			if (strcmp((const char *)USART_RX_BUF1, "pd1")==0)        pos_d=pos_d+50;
			
			if (strcmp((const char *)USART_RX_BUF1, "mode0")==0)     temp=0;
			if (strcmp((const char *)USART_RX_BUF1, "mode1")==0)     temp=1;
			if (strcmp((const char *)USART_RX_BUF1, "mode2")==0)     temp=2;
			
			if (strcmp((const char *)USART_RX_BUF1, "middle")==0)    middle=middle_check();
			if (strcmp((const char *)USART_RX_BUF1, "rat")==0)    rat=rat_check();
			if (strcmp((const char *)USART_RX_BUF1, "rat2")==0)   rat2_check();
			if (USART_RX_BUF1[1]=='l'||USART_RX_BUF1[2]=='l'||USART_RX_BUF1[3]=='l')
			{
			  if(USART_RX_BUF1[1]=='l') Moto_Target_angle=(float)USART_RX_BUF1[0]-48;
				if(USART_RX_BUF1[2]=='l') Moto_Target_angle=((float)USART_RX_BUF1[0]-48)*10+(float)USART_RX_BUF1[1]-48;
				if(USART_RX_BUF1[3]=='l') Moto_Target_angle=((float)USART_RX_BUF1[0]-48)*100+((float)USART_RX_BUF1[1]-48)*10+(int)USART_RX_BUF1[2]-48;
				temp=1;  
			}
			if (USART_RX_BUF1[1]=='r'||USART_RX_BUF1[2]=='r'||USART_RX_BUF1[3]=='r')
			{
			  if(USART_RX_BUF1[1]=='r') Moto_Target_angle=-((float)USART_RX_BUF1[0]-48);
				if(USART_RX_BUF1[2]=='r') Moto_Target_angle=-(((float)USART_RX_BUF1[0]-48)*10+(float)USART_RX_BUF1[1]-48);
				if(USART_RX_BUF1[3]=='r') Moto_Target_angle=-(((float)USART_RX_BUF1[0]-48)*100+((float)USART_RX_BUF1[1]-48)*10+(int)USART_RX_BUF1[2]-48);
				temp=1;  
			}
			
			USART_RX_STA1=0;
			LED=!LED;
			memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
		}
}
