/*
 * oled.i2c.c
 *
 *  Created on: Feb 12, 2026
 *      Author: julie
 */

#include "oled.i2c.h"


// IO direction setting
#define OLED_SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}
#define OLED_SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;}

//  IO operation function
#define OLED_IIC_SCL    PBout(8) //SCL
#define OLED_IIC_SDA    PBout(9) //SDA
#define OLED_READ_SDA   PBin(9)  //  Input SDA


#define DELAY_FOR_COUNT      10


static void Delay_For_Pin(uint8_t nCount)
{
    uint8_t i = 0;
    for(; nCount != 0; nCount--)
    {
        for (i = 0; i < DELAY_FOR_COUNT; i++);
    }
}


void OLED_I2C_Init(void)
{
		OLED_IIC_Init();
		OLED_Init();// OLED initialization

		OLED_Draw_Line("oled init success!", 1, true, true);
		printf("suis là");
}

/*
 * Initialisation
 */
void OLED_IIC_Init(void)
{
    RCC->APB2ENR |= 1 << 3;   // Enable peripheral IO PORTB clock first
    GPIOB->CRH &= 0XFFFFFF00; //Push-pull output
    GPIOB->CRH |= 0X00000033;
}



int OLED_IIC_Start(void)
{
    OLED_SDA_OUT(); // sda line output
    OLED_IIC_SDA = 1;
    if (!OLED_READ_SDA)
        return 0;
    OLED_IIC_SCL = 1;
    Delay_For_Pin(1);
    OLED_IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    if (OLED_READ_SDA)
        return 0;
    Delay_For_Pin(2);
    OLED_IIC_SCL = 0; // bus and prepare to send or receive data
    return 1;
}


void OLED_IIC_Stop(void)
{
    OLED_SDA_OUT(); // sda line output
    OLED_IIC_SCL = 0;
    OLED_IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    Delay_For_Pin(2);
    OLED_IIC_SCL = 1;
    Delay_For_Pin(1);
    OLED_IIC_SDA = 1; // Send I2C bus end signal
    Delay_For_Pin(2);
}

/*
 * envoie des ack
 */
int OLED_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    OLED_SDA_IN();
    OLED_IIC_SDA = 1;
    Delay_For_Pin(1);
    OLED_IIC_SCL = 1;
    Delay_For_Pin(1);
    while (OLED_READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 50)
        {
            OLED_IIC_Stop();
            return 0;
        }
        Delay_For_Pin(1);
    }
    OLED_IIC_SCL = 0; // Clock output 0
    return 1;
}


void OLED_IIC_Ack(void)
{
    OLED_IIC_SCL = 0;
    OLED_SDA_OUT();
    OLED_IIC_SDA = 0;
    Delay_For_Pin(1);
    OLED_IIC_SCL = 1;
    Delay_For_Pin(1);
    OLED_IIC_SCL = 0;
}


void OLED_IIC_NAck(void)
{
    OLED_IIC_SCL = 0;
    OLED_SDA_OUT();
    OLED_IIC_SDA = 1;
    Delay_For_Pin(1);
    OLED_IIC_SCL = 1;
    Delay_For_Pin(1);
    OLED_IIC_SCL = 0;
}

/*
 * Envoi et réception de données
 */

void OLED_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    OLED_SDA_OUT();
    OLED_IIC_SCL = 0; //Pull the clock low to start data transmission
    for (t = 0; t < 8; t++)
    {
        OLED_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        Delay_For_Pin(1);
        OLED_IIC_SCL = 1;
        Delay_For_Pin(1);
        OLED_IIC_SCL = 0;
        Delay_For_Pin(1);
    }
}

/*
 * ecrire des bites
 */

int OLED_i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i;
    if (!OLED_IIC_Start())
        return 1;
    OLED_IIC_Send_Byte(addr << 1);
    if (!OLED_IIC_Wait_Ack())
    {
        OLED_IIC_Stop();
        return 1;
    }
    OLED_IIC_Send_Byte(reg);
    OLED_IIC_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        OLED_IIC_Send_Byte(data[i]);
        if (!OLED_IIC_Wait_Ack())
        {
            OLED_IIC_Stop();
            return 0;
        }
    }
    OLED_IIC_Stop();
    return 0;
}

/*
 * lire un registre
 */
int OLED_i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!OLED_IIC_Start())
        return 1;
    OLED_IIC_Send_Byte(addr << 1);
    if (!OLED_IIC_Wait_Ack())
    {
        OLED_IIC_Stop();
        return 1;
    }
    OLED_IIC_Send_Byte(reg);
    OLED_IIC_Wait_Ack();
    OLED_IIC_Start();
    OLED_IIC_Send_Byte((addr << 1) + 1);
    OLED_IIC_Wait_Ack();
    while (len)
    {
        if (len == 1)
            *buf = OLED_IIC_Read_Byte(0);
        else
            *buf = OLED_IIC_Read_Byte(1);
        buf++;
        len--;
    }
    OLED_IIC_Stop();
    return 0;
}
/*
 * lire le bite
 */
uint8_t OLED_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    OLED_SDA_IN(); // SDA is set as input
    for (i = 0; i < 8; i++)
    {
        OLED_IIC_SCL = 0;
        Delay_For_Pin(2);
        OLED_IIC_SCL = 1;
        receive <<= 1;
        if (OLED_READ_SDA)
            receive++;
        Delay_For_Pin(2);
    }
    if (ack)
        OLED_IIC_Ack(); // Send ACK
    else
        OLED_IIC_NAck(); // Send nACK
    return receive;
}


unsigned char OLED_I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;

    OLED_IIC_Start();
    OLED_IIC_Send_Byte(I2C_Addr);
    res++;
    OLED_IIC_Wait_Ack();
    OLED_IIC_Send_Byte(addr);
    res++;
    OLED_IIC_Wait_Ack();

    OLED_IIC_Start();
    OLED_IIC_Send_Byte(I2C_Addr + 1);
    res++; // Entering receive mode
    OLED_IIC_Wait_Ack();
    res = OLED_IIC_Read_Byte(0);
    OLED_IIC_Stop();
    return res;
}


/*
 * lire plusieurs bites
 */
uint8_t OLED_OLED_IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t count = 0;

    OLED_IIC_Start();
    OLED_IIC_Send_Byte(dev);
    OLED_IIC_Wait_Ack();
    OLED_IIC_Send_Byte(reg);
    OLED_IIC_Wait_Ack();
    OLED_IIC_Start();
    OLED_IIC_Send_Byte(dev + 1);
    OLED_IIC_Wait_Ack();

    for (count = 0; count < length; count++)
    {

        if (count != length - 1)
            data[count] = OLED_IIC_Read_Byte(1);
        else
            data[count] = OLED_IIC_Read_Byte(0);
    }
    OLED_IIC_Stop();
    return count;
}



uint8_t OLED_OLED_IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{

    uint8_t count = 0;
    OLED_IIC_Start();
    OLED_IIC_Send_Byte(dev);
    OLED_IIC_Wait_Ack();
    OLED_IIC_Send_Byte(reg);
    OLED_IIC_Wait_Ack();
    for (count = 0; count < length; count++)
    {
        OLED_IIC_Send_Byte(data[count]);
        OLED_IIC_Wait_Ack();
    }
    OLED_IIC_Stop();

    return 1; //status == 0;
}



uint8_t OLED_IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    *data = OLED_I2C_ReadOneByte(dev, reg);
    return 1;
}



unsigned char OLED_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return OLED_OLED_IICwriteBytes(dev, reg, 1, &data);
}



uint8_t OLED_OLED_IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{

    uint8_t b;
    if (OLED_IICreadByte(dev, reg, &b) != 0)
    {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return OLED_IICwriteByte(dev, reg, b);
    }
    else
    {
        return 0;
    }
}



uint8_t OLED_IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    OLED_IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return OLED_IICwriteByte(dev, reg, b);
}

