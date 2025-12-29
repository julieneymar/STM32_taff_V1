/*
 * encoder.h
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#ifndef __ENCODER_H
#define __ENCODER_H

#include "AllHeader.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


#define ENCODER_TIM_PERIOD (u16)(65535)

typedef enum {
    MOTOR_ID_ML = 0,
    MOTOR_ID_MR,
    MAX_MOTOR
} Motor_ID;


int Read_Encoder(Motor_ID MYTIMX);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);


#endif


#endif /* INC_ENCODER_H_ */
