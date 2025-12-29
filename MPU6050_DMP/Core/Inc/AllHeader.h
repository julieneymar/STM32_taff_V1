/*
 * AllHeader.h
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef INC_ALLHEADER_H_
#define INC_ALLHEADER_H_


/**
* @par Copyright (C): 2016-2026, Shenzhen Yahboom Tech
* @file         // ALLHeader.h
* @author       // lly
* @version      // V1.0
* @date         // 240628
* @brief        // 相关所有的头文件 All related header files
* @details
* @par History  //
*
*/


#ifndef __ALLHEADER_H
#define __ALLHEADER_H


//C语言头文件  C language header file
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "key.h"
#include "pid_control.h"
#include "app_control.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"


// HAL library STM32 header file
#include "main.h"



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

#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "motor.h"
#include "encoder.h"
#include "IOI2C.h"
#include "mpu6050.h"
#include "pid_control.h"
#include "app_control.h"
#include "KF.h"
#include "delay.h"
#include "filtrer.h"



//引出的通用变量 General variables introduced
extern float Velocity_Left,Velocity_Right; 								//轮子的速度 The speed of the wheels
extern uint8_t GET_Angle_Way;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波  Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;     		//平衡倾角 平衡陀螺仪 转向陀螺仪 Balance tilt angle balance gyroscope steering gyroscope
extern int Motor_Left,Motor_Right;                 	  		//电机PWM变量 Motor PWM variable
extern int Temperature;                                		//温度变量 Temperature variable
extern float Acceleration_Z;                           		//Z轴加速度计  Z-axis accelerometer
extern int 	Mid_Angle;                          				//机械中值  Mechanical median
extern float Move_X,Move_Z;															//Move_X:前进速度（Forward speed）  Move_Z：转向速度(Turning speed)
extern float battery; 																	//电池电量	battery level
extern u8 lower_power_flag; 														//低电压标志,电压恢复标志 Low voltage sign, voltage recovery sign
extern u32 g_distance; 	//超声波距离值 Ultrasonic distance value
extern uint8_t angle_max;

//extern enCarState g_newcarstate; 												//小车状态标志 Car status indicator
extern u8 Stop_Flag;																		//停止标志  Stop sign
extern  float Car_Target_Velocity,Car_Turn_Amplitude_speed; //前进速度 旋转速度  Forward speed and rotational speed

//extern int Mid_Angle;																		//机械中值  Mechanical median


//extern Car_Mode mode; 																	//模式选择  Mode selection

#endif



#endif /* INC_ALLHEADER_H_ */
