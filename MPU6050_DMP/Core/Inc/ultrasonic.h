/*
 * ultrasonic.h
 *
 *  Created on: Feb 9, 2026
 *      Author: julie
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_
#include <iOI2C.h>
#include "main.h"
#include "ALLHeader.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TRIG_SIG PAout(0)

// Variables globales
extern volatile uint16_t TIM2CH2_CAPTURE_STA;
extern volatile uint16_t TIM2CH2_CAPTURE_VAL;
extern volatile uint32_t g_distance;

void Ultrasonic_Init(void);
void distance(void);
void APP_avoid(void);

void Get_Distane(void);


#endif /* INC_ULTRASONIC_H_ */
