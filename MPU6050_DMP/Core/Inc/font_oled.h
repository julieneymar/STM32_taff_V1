/*
 * Font_oled.h
 *
 *  Created on: Feb 9, 2026
 *      Author: julie
 */

#ifndef INC_FONT_OLED_H_
#define INC_FONT_OLED_H_

#include "AllHeader.h"

/**
 * @brief  Font structure used on my LCD libraries
 */
typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef_t;

extern FontDef_t *Font;

/**
 * @brief  7 x 10 pixels font size structure
 */
extern FontDef_t Font_7x10;

/**
 * @brief  11 x 18 pixels font size structure
 */
extern FontDef_t Font_11x18;

/**
 * @brief  16 x 26 pixels font size structure
 */
extern FontDef_t Font_16x26;


#endif /* INC_FONT_OLED_H_ */
