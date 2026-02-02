/*
 * pid_control.c
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef SRC_PID_CONTROL_C_
#define SRC_PID_CONTROL_C_


#include "pid_control.h"


float Balance_K = 2.0; //
float Velocity_K = 1.35;//
float Turn_K = 1.0; //

//Vertical loop PD control parameters
float Balance_Kp = 10000;// 3500                 10000
float Balance_Kd = 35; // 1.2                  40

//PI control parameters for speed loop
float Velocity_Kp=5000; // Range 0-72 6000    40   5000
float Velocity_Ki=30;  //kp/200  30       2        30

//Steering ring PD control parameters
float Turn_Kp=2000; // 200                       1400
float Turn_Kd=30; // 5                           30

//Forward speed
float Car_Target_Velocity=0; //0-10
//Rotation speed
float Car_Turn_Amplitude_speed= 0; //0-60

/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}


/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
**************************************************************************/
int Balance_PD(float Angle,float Gyro)
{
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias= Mid_Angle-Angle;
	 Gyro_bias=0-Gyro;
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100;


	 return balance;
}



int Velocity_PI(int encoder_left,int encoder_right)
{
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral;

	    Movement = Car_Target_Velocity;  // ← ICI !
		Encoder_Least =0-(encoder_left+encoder_right);
		Encoder_bias *= 0.84;
		Encoder_bias += Encoder_Least*0.16;
		Encoder_Integral +=Encoder_bias;
		Encoder_Integral=Encoder_Integral+Movement;
		if(Encoder_Integral>8000)  	Encoder_Integral=8000;
		if(Encoder_Integral<-8000)	  Encoder_Integral=-8000;
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;
		  // Reset si arrêt

		if(Turn_Off(Angle_Balance,battery)==1) Encoder_Integral=0;

	  return velocity;

}



float myTurn_Kd = 0;
int Turn_PD(float gyro)
{
	 static float Turn_Target,turn_PWM;
	 float Kp=Turn_Kp,Kd;

	 Turn_Target = Car_Turn_Amplitude_speed;
	 Kd=Turn_Kd;
     Kd=myTurn_Kd;

     turn_PWM=Turn_Target*Kp/100+gyro*Kd/100+Move_Z;

     return turn_PWM;
}


#endif /* SRC_PID_CONTROL_C_ */
