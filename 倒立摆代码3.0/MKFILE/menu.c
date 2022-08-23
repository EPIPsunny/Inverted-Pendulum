#include "stm32f10x.h"   // �൱��51��Ƭ���е�  #include <reg51.h>
#include "oled.h"
#include "delay.h"
#include "SOLGUI_Include.h"
#include "string.h"
#include "menu.h"
#include "key.h"
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "encoder.h"
#include "wifi.h"
#include "control.h"


MENU_PAGE FirstPage;
MENU_PAGE MODE1;
MENU_PAGE MODE2;


void menu_int(void)
{
   SOLGUI_Init(&FirstPage);
}

__M_PAGE(FirstPage,"First Page",PAGE_NULL,
{
	SOLGUI_Cursor(6,0,4);                                //�����ı�ѡ����
  //SOLGUI_Widget_GotoPage(0,&MODE1);                     //������ҳ��
	//SOLGUI_Widget_GotoPage(1,&MODE2);
	SOLGUI_Widget_OptionText(0,"roll=%f",roll);	
  SOLGUI_Widget_OptionText(1,"Velocity_Kd=%d" ,Velocity_Kd);
	SOLGUI_Widget_OptionText(2,"LastPWM=%d" ,LastPWM);
	//SOLGUI_Widget_OptionText(3,"Encoder_S=%d" ,Encoder_S);
	SOLGUI_Widget_OptionText(3,"Target_angle=%f" ,Moto_Target_angle);
	//SOLGUI_Widget_OptionText(2,"Velocity_Kp=%f" ,Velocity_Kp);
	//SOLGUI_Widget_OptionText(0,"meddle:%d" ,meddle);
});

__M_PAGE(MODE1,"MODE1",&FirstPage,                       //���������˵�
{
//  mode=1;                                               //ģʽ��־λ
//  SOLGUI_Cursor(6,0,7);                                 //
//	SOLGUI_Widget_OptionText(0, "date1: %d", date1);      //�����ı�ѡ��
//	SOLGUI_Widget_Spin(1,"page",INT32,100,0,&page);       //��ť���ں���
});
__M_PAGE( MODE2,"MODE2",&MODE1,                           //���������˵� **&MODE1**�ص㷵�� MODE1 ҳ��
{
	
	
		
});



