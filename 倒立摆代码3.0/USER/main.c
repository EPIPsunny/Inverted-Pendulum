#include "sys.h"
#include "delay.h"
#include "stm32f10x.h"  
#include "oled.h"
#include "menu.h"
#include "key.h"
#include "zimo.h"                                    //取字模文件
#include "SOLGUI_Include.h"
#include "led.h" 
#include "control.h"
#include "encoder.h"
//#include "usart.c"
#include "wifi.h"
#include "string.h"
#include "timer.h"
#include "adc.h"
#include "test.h"
/************************************************
采样一次
12.5v: -250,-400,10,360
12.4v: -250,-400,10,360   -250,-400,20,460  -250,-400,25,510
12.3v: -250,-400,10,360   -250,-400,15,400  -250,-400,20,460  
12.2v: -250,-400,10,440   -250,-400,15,420
12.1v: -250,-400,10,400   -250,-400,15,420
12.0v: -250,-400,10,400
采样三次
12.4v: -250,-400,3,400
************************************************/

unsigned int Adc2_org,Adc3_org; 		//adc原始值
float roll,d_roll,Last_roll;		//角度，角速度，上次角度
int SPEED_R;
int SPEED_L;
int Encoder_A_EXTI,Encoder_B_EXTI;
int lspeed,rspeed;
int Vertical_Kp=-250;//-500直立环
float Vertical_Kd=-400;//-800直立环
int Velocity_Kp=3;//10速度环
float Velocity_Ki=0.50;//0.50速度环
int Velocity_Kd=400;//400
int Vertical_out,Velocity_out,moto_pwm,LastPWM;
float middle=0;//处于直立状态下的(float)Adc2_org/(float)Adc3_org
float rat=11.827;//(float)4096/360;，比例系数rat=（（直立Adc2_org/Adc3_org）-（倒下Adc2_org/Adc3_org））/180, 为固定值
int Target_Speed=0;
unsigned char temp=5;
int pos_p=-70;//电机转动位置p=-60
float pos_i=0;//电机转动位置i，d
int pos_d=-150;//-230
float Moto_Target_angle=180;//电机旋转的目标角度
float rat2=3.001;//电机旋转的单位角度脉冲数
int Position_Zero=0;


 void JTAGDisable()                                  //禁用JTAG	
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}


 int main(void)
 {	
  LED_Init();
	delay_init();
	TIM1_PWM_Init(3999,9);
	JTAGDisable();  	                                  //禁用JTAG	很重要不得删去
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	Adc_Init(); //adc初始化
  Encoder_TIM4_Init();	//编码器初始化
  Encoder_TIM2_Init();  //编码器初始化
	INinit();   //tb6612逻辑初始化
	uart1_init(115200);
	memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//数组清零
	menu_int();
  OLED_Init();
	SOLGUI_Refresh();
	TIM3_Int_Init(1599,99);
	middle=(float)Adc2_org;
	while(1)
	{
	  SOLGUI_Menu_PageStage();		
    bluetooth_check();
	  SOLGUI_Refresh();       //OLED刷新		
	}
 }



