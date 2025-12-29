/*
 * led.c
 *
 *  Created on: Dec 29, 2025
 *      Author: julie
 */


#include "led.h"


void La_led (void){
	LED_ON ; // allume
    delay_us(5);
	LED_OFF;   // éteint
    delay_us(5);
}
