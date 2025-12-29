/*
 * key.c
 *
 *  Created on: Dec 23, 2025
 *      Author: julie
 */


/*
 * key.c
 *
 *  Created on: Dec 12, 2025
 *      Author: julie
 */

#include "key.h"
#include <stdio.h>


uint16_t g_key1_long_press = 0;


static uint8_t Key1_is_Press(void)
{
	if (!HAL_GPIO_ReadPin(BOUTTON_GPIO_Port, BOUTTON_Pin))
	{
		return BOUTTON_PRESS; //
	}
	return BOUTTON_RELEASE;   //
}


uint8_t Key1_Long_Press(uint16_t timeout)
{
	if (g_key1_long_press > 0)
	{
		if (g_key1_long_press < timeout * 100 + 2)
		{
			g_key1_long_press++;
			if (g_key1_long_press == timeout * 100 + 2)
			{
				return 1;
			}
			return 0;
		}
	}
	return 0;
}



uint8_t Key1_State(uint8_t mode)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == BOUTTON_PRESS)
	{
		if (key1_state < (mode + 1) * 2)
		{
			key1_state++;
		}
	}
	else
	{
		key1_state = 0;
		g_key1_long_press = 0;
	}
	if (key1_state == 2)
	{
		g_key1_long_press = 1;
		return BOUTTON_PRESS;
	}
	return BOUTTON_RELEASE;
}


/*********************************************END OF FILE**********************/

