/*
 * lec.h
 *
 *  Created on: Dec 29, 2025
 *      Author: julie
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "AllHeader.h"



#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB

#define LED_OFF   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_ON  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)


#define LED_OUT  PBout(3)

#endif /* INC_LED_H_ */
