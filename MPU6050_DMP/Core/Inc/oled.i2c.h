/*
 * oled.i2c.h
 *
 *  Created on: Feb 12, 2026
 *      Author: julie
 */

#include "AllHeader.h"

#ifndef INC_OLED_I2C_H_
#define INC_OLED_I2C_H_

void OLED_I2C_Init(void);



void OLED_IIC_Init(void);                  // Initialize IIC IO port
int OLED_IIC_Start(void);                  // Send IIC start signal
void OLED_IIC_Stop(void);                  //  Send IIC stop signal
void OLED_IIC_Send_Byte(uint8_t txd);           //   IIC sends a byte
uint8_t OLED_IIC_Read_Byte(unsigned char ack);  //  IIC reads a byte
int OLED_IIC_Wait_Ack(void);               // IIC waits for ACK signal
void OLED_IIC_Ack(void);                   //   IIC sends ACK signal
void OLED_IIC_NAck(void);                  // IIC does not send ACK signal

void OLED_IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t OLED_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);
unsigned char OLED_I2C_Readkey(unsigned char I2C_Addr);

unsigned char OLED_I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char OLED_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t OLED_IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t OLED_IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t OLED_IICwriteBit(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data);
uint8_t OLED_IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

int OLED_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int OLED_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


#endif /* INC_OLED_I2C_H_ */
