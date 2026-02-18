/*
 * ps2.h
 *
 *  Created on: Feb 11, 2026
 *      Author: julie
 */

#ifndef INC_PS2_H_
#define INC_PS2_H_
#include "AllHeader.h"

#define DI_PIN 	GPIO_PIN_14
#define DO_PIN 	GPIO_PIN_15
#define CS_PIN 	GPIO_PIN_12
#define CLK_PIN GPIO_PIN_13

#define DI_PORT 		GPIOB
#define DO_PORT 		GPIOB
#define CS_PORT 		GPIOB
#define CLK_PORT 	GPIOB

#define DI   PBin(14) //

#define DO_H PBout(15)=1       //  Command bit high
#define DO_L PBout(15)=0       //Command bit low

#define CS_H PBout(12)=1       //  CS pull high
#define CS_L PBout(12)=0       //  CS pull low

#define CLK_H PBout(13)=1      //  Clock pull high
#define CLK_L PBout(13)=0      // Clock pull low



//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //  Right joystick X-axis data
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;

void PS2_Init(void);
uint8_t PS2_RedLight(void);   //Determine whether it is red light mode
void PS2_ReadData(void); //  Read handle data
void PS2_Cmd(uint8_t CMD);		  //Sending commands to the controller
uint8_t PS2_DataKey(void);		  // Read the key value
uint8_t PS2_AnologData(uint8_t button); //  Get the analog value of a joystick
void PS2_ClearData(void);	  //  Clear the data buffer
void PS2_Vibration(uint8_t motor1, uint8_t motor2);// Vibration setting motor1 0xFF on, others off, motor2 0x40~0xFF

void PS2_EnterConfing(void);	 // Enter configuration
void PS2_TurnOnAnalogMode(void); // Send analog value
void PS2_VibrationMode(void);    // Vibration setting
void PS2_ExitConfing(void);	     // Complete configuration
void PS2_SetInit(void);		     // Configuration initialization

/*

// Macros
#define CS_H        HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)
#define CS_L        HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)
#define CLK_H       HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET)
#define CLK_L       HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET)
#define DO_H        HAL_GPIO_WritePin(DO_PORT, DO_PIN, GPIO_PIN_SET)
#define DO_L        HAL_GPIO_WritePin(DO_PORT, DO_PIN, GPIO_PIN_RESET)
#define DI          HAL_GPIO_ReadPin(DI_PORT, DI_PIN)

*/
#endif /* INC_PS2_H_ */
