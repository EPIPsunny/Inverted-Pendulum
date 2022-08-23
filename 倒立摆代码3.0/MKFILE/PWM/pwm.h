#ifndef __PWM_H

#define __PWM_H

 #include "sys.h"

#define PWM1 TIM3->CCR1        //模板上的PWM4    PA6
#define PWM2 TIM3->CCR2   //模板上的PWM7        PA7
#define PWM3 TIM3->CCR3     ////模板上的PWM3      PB0
#define PWM4 TIM3->CCR4     //模板上的PWM0       PB1

void TIM3_PWM_Init(u16 arr,u16 psc);

#define Laser PAout(12)






#endif

