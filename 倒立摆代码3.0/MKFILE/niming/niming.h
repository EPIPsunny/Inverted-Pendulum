#ifndef __NIMING_H
#define __NIMING_H


typedef unsigned char u8;

void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void send_data_int(float data0,float data1,float data2);//漂自邓学长风力摆源码
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
#endif
