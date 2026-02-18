/*
 * uart5.h
 *
 *  Created on: Feb 11, 2026
 *      Author: julie
 */

#ifndef INC_UART5_H_
#define INC_UART5_H_

#include "AllHeader.h"

void UART5_Send_U8(uint8_t ch);
void UART5_Send_ArrayU8(uint8_t *buffer, uint16_t length);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART5_H_ */
