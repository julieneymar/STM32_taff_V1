/*
 * key.h
 *
 *  Created on: Dec 23, 2025
 *      Author: julie
 */


#ifndef INC_KEY_H_
#define INC_KEY_H_

#include "stm32f1xx_hal.h"

#define BOUTTON_Pin         GPIO_PIN_8
#define BOUTTON_GPIO_Port   GPIOA

#define BOUTTON_PRESS       1
#define BOUTTON_RELEASE     0



uint8_t Key1_State(uint8_t mode);
uint8_t Key1_Long_Press(uint16_t timeout);

#endif /* INC_KEY_H_ */
