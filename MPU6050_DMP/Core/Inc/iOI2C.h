/*
 * IOI2C.h
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#ifndef INC_IOI2C_H_
#define INC_IOI2C_H_


#include "stm32f1xx_hal.h"

#include "AllHeader.h"

// IO port operation macro definition
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

//IO port address mapping
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08


#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  // Output
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  // Input

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  // Output
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  // Input

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // Output
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // Input

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  // Output
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  // Input

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  // Output
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  // Input

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  // Output
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  // Input

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  // Output
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  // Input

#define I2C_SCL_PORT    GPIOB
#define I2C_SCL_PIN     GPIO_PIN_11

#define I2C_SDA_PORT    GPIOB
#define I2C_SDA_PIN     GPIO_PIN_10

#define IIC_SCL    PBout(11) // SCL
#define IIC_SDA    PBout(10) // SDA
#define READ_SDA   PBin(10)  // Lecture SDA




// ============================================
// PROTOTYPES
// ============================================

void IIC_MPU6050_Init(void);
int IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
int IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void SDA_OUT(void);
void SDA_IN(void);


unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);


#endif /* INC_IOI2C_H_ */
