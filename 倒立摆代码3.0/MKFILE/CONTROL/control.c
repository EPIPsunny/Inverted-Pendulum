#include "sys.h"
#include "delay.h"
#include "stm32f10x.h"  
#include "oled.h"
#include "menu.h"                                  
#include "SOLGUI_Include.h"
#include "led.h" 
#include "control.h"
#include "encoder.h"
//#include "usart.c"
#include "wifi.h"
#include "string.h"
#include "timer.h"
#include "adc.h"


static float Position_Bias,Last_Position;


void INinit(void)   //tb6612逻辑初始化
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;				 //使用PD6
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}

void TIM1_PWM_Init(unsigned int arr,unsigned int psc)  //PA8 PA11
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitTypeStrue;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;				 //使用pa7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.7
	
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
	
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwm模式2，大有效，小无效
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//有效极性为高
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC1Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwm模式2，大有效，小无效
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//有效极性为高
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC4Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//高级定时器专属--MOE主输出使能
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//使能自动重装载的预装载
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//使能自动重装载的预装载
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);   /*使能TIMx在ARR上的预装载寄存器*/
	
	TIM_Cmd(TIM1,ENABLE);
}

void right_back(unsigned int speed)
{
  TIM_SetCompare4(TIM1,speed);
	left_motor_back;
}

void left_back(unsigned int speed)
{
  TIM_SetCompare1(TIM1,speed);
	right_motor_back;
}

void right_forward(unsigned int speed)
{
  TIM_SetCompare4(TIM1,speed);
	left_motor_go;
}

void left_forward(unsigned int speed)
{
  TIM_SetCompare1(TIM1,speed);
	right_motor_go;
}



void LoadPWM(int pwm)
{
  if(pwm>=0)
	{
		if(pwm>7200) pwm=7200;
	  left_back(pwm);
		right_back(pwm);
	}
	if(pwm<0)
	{
	  pwm=-pwm;
		if(pwm>7200) pwm=7200;
		left_forward(pwm);
		right_forward(pwm);
	}
}
	

/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/


int Vertical(float Angle)
{  
   float Bias;                       
	 static float Last_Bias,D_Bias;   
	 int balance;                   
	 Bias=Angle-0;          
	 D_Bias=Bias-Last_Bias;            
	 balance=-Vertical_Kp*Bias-D_Bias*Vertical_Kd;   
   Last_Bias=Bias;                  
	 return balance;
}



/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
//int Velocity(int Target,int encoder_left,int encoder_right)
//{
//	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
//	float a=0.7;
//	
//	//1.计算速度偏差
//	Encoder_Err=((encoder_left+encoder_right)-Target);//舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
//	//2.对速度偏差进行低通滤波
//	//low_out=(1-a)*Ek+a*low_out_last;
//	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
//	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
//	//3.对速度偏差积分，积分出位移
//	Encoder_S+=EnC_Err_Lowout;
//	//4.积分限幅
//	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
//	
//	//5.速度环控制输出计算
//	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
//	return PWM_out;
//}

int Velocity(int encoder_right)
{  
  static float Position_PWM,Position_Differential;
	static float Position_Least;
  Position_Least =encoder_right-Position_Zero;       
  Position_Bias *=0.8;		   
  Position_Bias += Position_Least*0.2;	        
	Position_Differential=Position_Bias-Last_Position;
	Last_Position=Position_Bias;
  Position_PWM=Position_Bias*Velocity_Kp+Position_Differential*Velocity_Kd;
	return Position_PWM;

}



/*****************************************************
函数名：PID_Pos_PosCalc(int moto_target_position_times,int moto_now_position_times)
输入：转到目标位置需经过的脉冲数，目前经过的脉冲数
输出：电机驱动pwm输出值
*******************************************************/

int PID_Pos_PosCalc(int moto_target_position_times,int moto_now_position_times)  //电机旋转角度pid
{
	static int pos_SumError,pos_LastError,iError,dError;
	iError = moto_target_position_times - moto_now_position_times;        				// 偏差
	pos_SumError += iError;				    				// 积分
	if(pos_SumError > 2000.0)								//积分限幅1000
		pos_SumError = 2000.0;
	if(pos_SumError < -2000.0)
		pos_SumError = -2000.0;	
	dError = iError - pos_LastError; 						// 当前微分
	pos_LastError = iError;
	
	return (pos_p * iError + pos_i * pos_SumError + pos_d * dError);	//返回计算值
}

/*****************************************************
函数名：moto_position_count(Target_angle,Moto_rat)
输入：电机旋转目标角度，电机每旋转一度的脉冲数
输出：电机转到目标角度所应经过的脉冲数
备注：该函数默认将上次目标角度作为目前静止状态下的实际角度
*******************************************************/
int moto_position_count(float Target_angle,int Moto_rat)
{
  static float Last_angle=0;
	float Angle_error;
	int Angle_error_times;
	Angle_error=Target_angle-Last_angle;
	Angle_error_times=Angle_error*Moto_rat;
	Last_angle=Target_angle;
	return Angle_error_times;
}



void TIM3_IRQHandler(void)
{
	static unsigned char ReadSpeed_5=0;
	static unsigned char FlagSpeed=0;
	static float Last_roll=0;
	static int speed_count=0;
	static unsigned int count=0;
  if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{
		Adc2_org=Get_Adc_Average(ADC_Channel_2,3);
		//Adc3_org=Get_Adc_Average(ADC_Channel_3,3);
		roll=((float)Adc2_org-middle)/rat;
		if(roll>180) roll=-360+roll;
		if(roll<-180) roll=360+roll;
		d_roll=roll-Last_roll;
		Last_roll=roll;
	  //SPEED_L=-Read_Speed(2);

		//Velocity_out=Velocity(0,0,SPEED_R);//速度环入口：左右实际速度
		//Vertical_out=Vertical(0,roll,d_roll);//直立环入口：期望角度、真实角度、真实角速度
		//LastPWM=Vertical_out-Vertical_Kp*Velocity_out;
		//printf("%d\r\n",Adc2_org);
    if(temp!=2) SPEED_R=Read_Speed(4);
			

		if(temp==0) 
		{
			LoadPWM(0);
			LastPWM=0;
			FlagSpeed=0;
			TIM_SetCounter(TIM2,0);
			Position_Zero=0;
			Position_Bias=0;
			Last_Position=0;
		}
		if(temp==1)
		{
//			ReadSpeed_5++;
//		  if(ReadSpeed_5>5)
//		  {
//		    ReadSpeed_5=0;
//		  }
			
			speed_count=speed_count+SPEED_R;
		  moto_pwm=PID_Pos_PosCalc(rat2*Moto_Target_angle,speed_count);//moto_position_count(Moto_Target_angle,rat2)
			if(moto_pwm>1000) moto_pwm=1000;
			if(moto_pwm<-1000) moto_pwm=-1000;
			LoadPWM(moto_pwm);  //moto_pwm
			count++;
			if(count>400)//经过2.5秒后停转
			{
			  temp=0;
				count=0;
				LoadPWM(0);
				speed_count=0;
				ReadSpeed_5=0;
				SPEED_R=0;
				TIM_SetCounter(TIM2,0);
				
			}
		}
    if(temp==2)        
		{
			if(FlagSpeed==0)
			{
				Position_Zero=Mind_Read_Speed(4);
				FlagSpeed=1;
			}
		  if(ReadSpeed_5>4)
		  {
		    SPEED_R=Mind_Read_Speed(4);
		    ReadSpeed_5=0;
		  }
			ReadSpeed_5++;
			Vertical_out=Vertical(roll);
			Velocity_out=Velocity(SPEED_R);//速度环入口：左右实际速度
			LastPWM=Vertical_out-Velocity_out;
			if(roll<45&&roll>-45)  
			{
				if(LastPWM>7200) LastPWM=7200;
				if(LastPWM<-7200) LastPWM=-7200;
				LoadPWM(LastPWM);
			}
			else                   LoadPWM(0);
		}
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	  
	}
}
