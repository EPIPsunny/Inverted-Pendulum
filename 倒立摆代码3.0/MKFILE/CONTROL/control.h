#ifndef __CONTROL_H
#define __CONTROL_H	 
#include "sys.h"



#define AIN1  PBout(12)
#define AIN2  PBout(13)
#define BIN1  PBout(14)
#define BIN2  PBout(15)

#define right_motor_stops AIN1 = 0, AIN2 = 0
#define left_motor_stops BIN1 = 0, BIN2 = 0

#define right_motor_go     AIN1 = 0, AIN2 = 1
#define right_motor_back   AIN1 = 1, AIN2 = 0
#define left_motor_go		   BIN1 = 0, BIN2 = 1
#define left_motor_back	   BIN1 = 1, BIN2 = 0
					


void INinit(void);   //tb6612逻辑初始化
void TIM1_PWM_Init(unsigned int arr,unsigned int psc);  //PA8 PA11
void left_forward(unsigned int speed);
void right_forward(unsigned int speed);
void right_back(unsigned int speed);
void left_back(unsigned int speed);

int Vertical(float Angle);//直立环
void LoadPWM(int pwm);
int Velocity(int encoder_right);
//int balance(float Angle);


extern int Vertical_Kp;//直立环
extern float Vertical_Kd;//直立环
extern int Velocity_Kp;//速度环
extern float Velocity_Ki;//速度环
extern int Velocity_Kd;
extern unsigned int Adc2_org,Adc3_org; 		//adc原始值
extern float roll,d_roll,Last_roll;		//旋转角度比例
extern int SPEED_R;
extern int SPEED_L;
extern int Vertical_out,Velocity_out,moto_pwm,LastPWM;
extern int Target_Speed;
extern float middle;
extern float rat;
extern unsigned char temp;
extern int pos_p;//电机转动位置p
extern float pos_i;//电机转动位置i，d
extern int pos_d;
extern float Moto_Target_angle;
extern float rat2;
extern int Position_Zero;

#endif
