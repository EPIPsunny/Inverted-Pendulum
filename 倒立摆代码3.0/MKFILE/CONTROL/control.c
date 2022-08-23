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


void INinit(void)   //tb6612�߼���ʼ��
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;				 //ʹ��PD6
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
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
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;				 //ʹ��pa7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.7
	
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwmģʽ2������Ч��С��Ч
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//��Ч����Ϊ��
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC1Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_OCInitTypeStrue.TIM_OCMode=TIM_OCMode_PWM1;//pwmģʽ2������Ч��С��Ч
	TIM_OCInitTypeStrue.TIM_OCPolarity=TIM_OCPolarity_High;//��Ч����Ϊ��
	TIM_OCInitTypeStrue.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitTypeStrue.TIM_Pulse=0;
	TIM_OC4Init(TIM1,&TIM_OCInitTypeStrue);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//�߼���ʱ��ר��--MOE�����ʹ��
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ���Զ���װ�ص�Ԥװ��
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ���Զ���װ�ص�Ԥװ��
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);   /*ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���*/
	
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
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
���ڣ�ֱ�������
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
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
*********************/
//int Velocity(int Target,int encoder_left,int encoder_right)
//{
//	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
//	float a=0.7;
//	
//	//1.�����ٶ�ƫ��
//	Encoder_Err=((encoder_left+encoder_right)-Target);//��ȥ���--�ҵ���⣺�ܹ����ٶ�Ϊ"0"�ĽǶȣ����ǻ�е��ֵ��
//	//2.���ٶ�ƫ����е�ͨ�˲�
//	//low_out=(1-a)*Ek+a*low_out_last;
//	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
//	EnC_Err_Lowout_last=EnC_Err_Lowout;//��ֹ�ٶȹ����Ӱ��ֱ����������������
//	//3.���ٶ�ƫ����֣����ֳ�λ��
//	Encoder_S+=EnC_Err_Lowout;
//	//4.�����޷�
//	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
//	
//	//5.�ٶȻ������������
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
��������PID_Pos_PosCalc(int moto_target_position_times,int moto_now_position_times)
���룺ת��Ŀ��λ���辭������������Ŀǰ������������
������������pwm���ֵ
*******************************************************/

int PID_Pos_PosCalc(int moto_target_position_times,int moto_now_position_times)  //�����ת�Ƕ�pid
{
	static int pos_SumError,pos_LastError,iError,dError;
	iError = moto_target_position_times - moto_now_position_times;        				// ƫ��
	pos_SumError += iError;				    				// ����
	if(pos_SumError > 2000.0)								//�����޷�1000
		pos_SumError = 2000.0;
	if(pos_SumError < -2000.0)
		pos_SumError = -2000.0;	
	dError = iError - pos_LastError; 						// ��ǰ΢��
	pos_LastError = iError;
	
	return (pos_p * iError + pos_i * pos_SumError + pos_d * dError);	//���ؼ���ֵ
}

/*****************************************************
��������moto_position_count(Target_angle,Moto_rat)
���룺�����תĿ��Ƕȣ����ÿ��תһ�ȵ�������
��������ת��Ŀ��Ƕ���Ӧ������������
��ע���ú���Ĭ�Ͻ��ϴ�Ŀ��Ƕ���ΪĿǰ��ֹ״̬�µ�ʵ�ʽǶ�
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

		//Velocity_out=Velocity(0,0,SPEED_R);//�ٶȻ���ڣ�����ʵ���ٶ�
		//Vertical_out=Vertical(0,roll,d_roll);//ֱ������ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
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
			if(count>400)//����2.5���ͣת
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
			Velocity_out=Velocity(SPEED_R);//�ٶȻ���ڣ�����ʵ���ٶ�
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
