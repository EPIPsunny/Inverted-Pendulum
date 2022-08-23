#include "sys.h"
#include "delay.h"
#include "stm32f10x.h"  
#include "oled.h"
#include "menu.h"
#include "key.h"
#include "zimo.h"                                    //ȡ��ģ�ļ�
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
����һ��
12.5v: -250,-400,10,360
12.4v: -250,-400,10,360   -250,-400,20,460  -250,-400,25,510
12.3v: -250,-400,10,360   -250,-400,15,400  -250,-400,20,460  
12.2v: -250,-400,10,440   -250,-400,15,420
12.1v: -250,-400,10,400   -250,-400,15,420
12.0v: -250,-400,10,400
��������
12.4v: -250,-400,3,400
************************************************/

unsigned int Adc2_org,Adc3_org; 		//adcԭʼֵ
float roll,d_roll,Last_roll;		//�Ƕȣ����ٶȣ��ϴνǶ�
int SPEED_R;
int SPEED_L;
int Encoder_A_EXTI,Encoder_B_EXTI;
int lspeed,rspeed;
int Vertical_Kp=-250;//-500ֱ����
float Vertical_Kd=-400;//-800ֱ����
int Velocity_Kp=3;//10�ٶȻ�
float Velocity_Ki=0.50;//0.50�ٶȻ�
int Velocity_Kd=400;//400
int Vertical_out,Velocity_out,moto_pwm,LastPWM;
float middle=0;//����ֱ��״̬�µ�(float)Adc2_org/(float)Adc3_org
float rat=11.827;//(float)4096/360;������ϵ��rat=����ֱ��Adc2_org/Adc3_org��-������Adc2_org/Adc3_org����/180, Ϊ�̶�ֵ
int Target_Speed=0;
unsigned char temp=5;
int pos_p=-70;//���ת��λ��p=-60
float pos_i=0;//���ת��λ��i��d
int pos_d=-150;//-230
float Moto_Target_angle=180;//�����ת��Ŀ��Ƕ�
float rat2=3.001;//�����ת�ĵ�λ�Ƕ�������
int Position_Zero=0;


 void JTAGDisable()                                  //����JTAG	
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}


 int main(void)
 {	
  LED_Init();
	delay_init();
	TIM1_PWM_Init(3999,9);
	JTAGDisable();  	                                  //����JTAG	����Ҫ����ɾȥ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	Adc_Init(); //adc��ʼ��
  Encoder_TIM4_Init();	//��������ʼ��
  Encoder_TIM2_Init();  //��������ʼ��
	INinit();   //tb6612�߼���ʼ��
	uart1_init(115200);
	memset(USART_RX_BUF1,0,sizeof(USART_RX_BUF1));//��������
	menu_int();
  OLED_Init();
	SOLGUI_Refresh();
	TIM3_Int_Init(1599,99);
	middle=(float)Adc2_org;
	while(1)
	{
	  SOLGUI_Menu_PageStage();		
    bluetooth_check();
	  SOLGUI_Refresh();       //OLEDˢ��		
	}
 }



