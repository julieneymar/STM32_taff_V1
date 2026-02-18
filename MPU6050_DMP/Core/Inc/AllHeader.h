/*
 * AllHeader.h
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef INC_ALLHEADER_H_
#define INC_ALLHEADER_H_





#ifndef __ALLHEADER_H
#define __ALLHEADER_H


//C language header file
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "key.h"
#include "pid_control.h"
#include "app_control.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "adc.h"
#include "ultrasonic.h"
#include <iOI2C.h>
#include <oled.h>
#include <font_oled.h>
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "pid_control.h"
#include "app_control.h"
#include "KF.h"
#include "delay.h"
#include "filtrer.h"
#include "uart5.h"
#include "ps2.h"
#include "app_ps2.h"
#include "oled.i2c.h"



// HAL library STM32 header file
#include "main.h"

extern TIM_HandleTypeDef htim2;


#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif





//Mpu6050





// General variables introduced
extern float Velocity_Left,Velocity_Right; 								// The speed of the wheels
extern uint8_t GET_Angle_Way;                             //  Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;     		// Balance tilt angle balance gyroscope steering gyroscope
extern int Motor_Left,Motor_Right;                 	  		// Motor PWM variable
extern int Temperature;                                		//Temperature variable
extern float Acceleration_Z;                           		//  Z-axis accelerometer
extern int 	Mid_Angle;                          				//  Mechanical median
extern float Move_X,Move_Z;															//（Forward speed）  Move_Z：(Turning speed)
extern float battery; 																	//battery level
extern uint8_t lower_power_flag; 														// Low voltage sign, voltage recovery sign
extern uint8_t angle_max;

extern char showbuf[20];

//extern enCarState g_newcarstate; 												// Car status indicator
extern uint8_t Stop_Flag;
extern  float Car_Target_Velocity,Car_Turn_Amplitude_speed; //  Forward speed and rotational speed



#endif



#endif /* INC_ALLHEADER_H_ */
