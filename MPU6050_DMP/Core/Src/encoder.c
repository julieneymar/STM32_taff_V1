/*
 * encoder.c
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */


#include "encoder.h"

void Encoder_Init_TIM3(void)
{
	TIM3->CNT = 0x0;
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);

}


void Encoder_Init_TIM4(void)
{
	TIM4->CNT = 0x0;
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}


int Read_Encoder(Motor_ID MYTIMX)
{
   int Encoder_TIM;
   switch(MYTIMX)
	 {
		 case MOTOR_ID_ML:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;
		 case MOTOR_ID_MR:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
		 default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
