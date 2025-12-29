/*
 * motor.c
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */


#include "motor.h"

uint8_t angle_max;

float Velocity_Left,Velocity_Right;	//  Wheel speed (mm/s)

#define MOTOR_IGNORE_PULSE (1300)//  Dead zone   1450 25Khz   This value needs to be fine-tuned in static state

int PWM_Ignore(int pulse)
{
	if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
  if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
	return pulse;
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left：Left wheel PWM；motor_right：Right wheel PWM
Output  : none
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
	if(motor_left == 0)// stop
	{
		L_PWMA = 0;
		L_PWMB = 0;
	}
	if(motor_right == 0)
	{
		R_PWMA = 0;
		R_PWMB = 0;
	}

	//  Left wheel
  if(motor_left>0)
	{
		L_PWMB = myabs(motor_left);
		L_PWMA = 0;
	}
	else
	{
		L_PWMB = 0;
		L_PWMA = myabs(motor_left);
	}

	// Right wheel
	if(motor_right>0)
	{
		R_PWMA = myabs(motor_right);
		R_PWMB = 0;
	}
	else // Back
	{
		R_PWMA = 0;
		R_PWMB = myabs(motor_right);
	}

}


/**************************************************************************
Function: PWM limiting range
Input   : IN：Input  max：Maximum value  min：Minimum value
Output  : Output
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}


/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	float Rotation_Speed_L,Rotation_Speed_R;						 //Motor speed=Encoder reading (5ms each time) * Reading frequency/harmonics/reduction ratio/Encoder accuracy
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67/10;		 //Calculate the encoder speed=rotational speed * circumference/10 and convert it to cm
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67/10;	 //Calculate the encoder speed=rotational speed * circumference/10 and convert it to cm

}



/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle：Car inclination；voltage：Voltage
Output  : 1：abnormal；0：normal
**************************************************************************/

uint8_t Turn_Off(float angle, float voltage)
{
    if(myabs((int)angle - Mid_Angle) > 40)
        return 1;

    if(voltage < 9.6)  // ⭐ Seuil du code qui marche
        return 1;

    return 0;
}


void Motor_start(void)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

}

