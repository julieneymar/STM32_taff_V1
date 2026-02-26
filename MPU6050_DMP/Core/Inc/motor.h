/*
 * motor.h
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#ifndef __MOTOR_H
#define __MOTOR_H

#include "AllHeader.h"



#define L_PWMA   TIM8->CCR1  //PC6
#define L_PWMB   TIM8->CCR2  //PC7

#define R_PWMA   TIM8->CCR3  //PC8
#define R_PWMB   TIM8->CCR4  //PC9


extern TIM_HandleTypeDef htim8;

void Motor_start(void);
#endif

#ifndef __APP_MOTOR_H_
#define __APP_MOTOR_H_

#include "ALLHeader.h"

#define PI 3.14159265							// PI π
#define EncoderMultiples   4.0 		//  Encoder multiples
#define Encoder_precision  11.0 	// Encoder precision 11 lines
#define Reduction_Ratio  30.0			//  Reduction ratio 30
#define Perimeter  210.4867 			// Perimeter, unit mm


void Set_Pwm(int motor_left,int motor_right);
int PWM_Limit(int IN,int max,int min);

void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
uint8_t Turn_Off(float angle, float voltage);

int PWM_Ignore(int pulse);


#endif



#endif /* INC_MOTOR_H_ */
