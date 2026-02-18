/*
 * app_control.h
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef INC_APP_CONTROL_H_
#define INC_APP_CONTROL_H_

#ifndef __APP_CONTROL_H_
#define __APP_CONTROL_H_


#include "ALLHeader.h"



#define MPU6050_INT PAin(12)   //  PA12 is connected to the interrupt pin of MPU6050

void Get_Angle(uint8_t way);
uint16_t get_time_int(void);
void delay_time_int(uint16_t time);
void set_time_int(uint16_t time);

//extern volatile uint8_t Start_Flag;


int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Read_MPU6050_Burst(float *gyro_x, float *gyro_y, float *gyro_z,
                        float *accel_x, float *accel_y, float *accel_z);

void Serial_Controle(void);
void Ultrasonic(void);

#endif

#endif /* INC_APP_CONTROL_H_ */
