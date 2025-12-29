/*
 * pid_control.h
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "ALLHeader.h"
// ===== PID variables (visibles partout) =====
extern float Balance_Kp;
extern float Balance_Kd;

extern float Velocity_Kp;
extern float Velocity_Ki;

extern float Turn_Kp;
extern float Turn_Kd;



int Balance_PD(float Angle,float Gyro);
int Velocity_PI(int encoder_left,int encoder_right);
int Turn_PD(float gyro);
int myabs(int a);

#endif



#endif /* INC_PID_CONTROL_H_ */
