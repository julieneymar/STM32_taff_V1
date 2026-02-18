/*
 * adc.h
 *
 *  Created on: Feb 9, 2026
 *      Author: julie
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

// Dans battery.h
#include "main.h"
#include <stdio.h>
#include<stdlib.h>
#include <string.h>
// Définitions des paramètres ADC
#define BAT_ADC_CH      ADC_CHANNEL_5     // Canal ADC pour la batterie
#define ADC_TIMEOUT     100               // Timeout en ms

extern ADC_HandleTypeDef hadc1;

uint16_t Battery_Get_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint8_t times);
float Get_Measure_Voltage(ADC_HandleTypeDef* hadc);
float Get_Battery_Voltage(ADC_HandleTypeDef* hadc);
void Battery_ADC_Init_DMA(ADC_HandleTypeDef* hadc);



#endif /* INC_ADC_H_ */
