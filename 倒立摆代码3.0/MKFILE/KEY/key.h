#ifndef __KEY_H

#define __KEY_H

#include "sys.h"

//----【按键个数】
#define KEY_NUM 6
//----【长按时间参数】
#define Long_Press_Time 100


//###########################【内部】##############################
typedef struct __KEY_T
{
	u8 (*Is_Key_Down)(void);		//硬件检测函数
	
	u8 Trg;							//触发 
	
	u8 Cont;						//按住
	
	u16 fliter_cont;
	
	u8 S_Value;						//短按键值
	
	u8 L_Value;						//长按键值

}KEY_T;


//----【键值定义】
#define KEY_NULL 	0x00 	//未按下

#define KEY_R_S 	0x10	//右键短按

#define KEY_R_L 	0x11	//右键长按

#define KEY_D_S 	0x20	//下键短按

#define KEY_D_L 	0x21	//下键长按

#define KEY_L_S 	0x30	//左键短按

#define KEY_L_L 	0x31	//左键长按

#define KEY_OK_S 	0x40	//OK键短按

#define KEY_OK_L 	0x41	//OK键长按

#define KEY_U_S 	0x50	//上键短按

#define KEY_U_L 	0x51	//上键长按

#define KEY_BK_S 	0x60	//Back键短按

#define KEY_BK_L 	0x61	//Back键长按




#define KEY_UP GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)				//上

#define KEY_DO GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)				//下

#define KEY_LE GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)				//左

#define KEY_BA GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)				//右

#define KEY_OK GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)				//OK

#define KEY_RI GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)				//返回


void KEY_Init(void);//IO初始化
void KeyRead(KEY_T *key);
	
u8 Key_GetValue(void);			//方向键及功能键获取键值【非阻塞，必须被周期性的调用(主函数或定时器)】





#endif
