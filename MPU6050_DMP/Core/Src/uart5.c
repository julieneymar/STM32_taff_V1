/*
 * uart5.c
 *
 *  Created on: Feb 11, 2026
 *      Author: julie
 */
#include "uart5.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

extern uint8_t UART5_RxByte;


// envoi un octet
void UART5_Send_U8(uint8_t ch)
{
    HAL_UART_Transmit(&huart5, &ch, 1, HAL_MAX_DELAY);
}


// envoie un tableau

void UART5_Send_ArrayU8(uint8_t *buffer, uint16_t length)
{
    HAL_UART_Transmit(&huart5, buffer, length, HAL_MAX_DELAY);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        // Octet reçu dans UART5_RxByte
        HAL_UART_Transmit(&huart1, &UART5_RxByte, 1, HAL_MAX_DELAY);

        // Relancer la réception
        HAL_UART_Receive_IT(&huart5, &UART5_RxByte, 1);
    }
}

