/*
 * IOI2C.c
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */


#include "../../../../MPU6050_DMP/Core/Inc/IOI2C.h"

//---------------- Fonctions de configuration SDA----------------/


void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**************************************************************************
Function: IIC pin initialization
Input   : none
Output  : none
**************************************************************************/
// -------------------------Initialisation du bus I2C logiciel----------

void IIC_MPU6050_Init(void)
{
    SDA_OUT();
    IIC_SCL = 1;
    IIC_SDA = 1;
}

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : 1
**************************************************************************/
int IIC_Start(void)
{
	SDA_OUT();     // sda line output
	IIC_SDA=1;
	if(!READ_SDA)return 0;
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0; //START:when CLK is high,DATA change form high to low
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;// Clamp the I2C bus and prepare to send or receive data
	return 1;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
**************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();// sda line output
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1;
	IIC_SDA=1;// Send I2C bus end signal
	delay_us(1);
}

/**************************************************************************
Function: IIC wait the response signal
Input   : none
Output  : 0：No response received；1：Response received

**************************************************************************/
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA is set as input
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//0 Clock output 0
	return 1;
}

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
**************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**************************************************************************
Function: IIC sends a byte
Input   : txd：Byte data sent
Output  : none
**************************************************************************/
void IIC_Send_Byte(u8 txd)
{
    u8 t;
	  SDA_OUT();
    IIC_SCL=0;//Pull the clock low to start data transmission
    for(t=0;t<8;t++)
    {
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1;
			delay_us(1);
			IIC_SCL=1;
			delay_us(1);
			IIC_SCL=0;
			delay_us(1);
    }
}

/**************************************************************************
Function: IIC write data to register
Input   : addr：Device address；reg：Register address；len;Number of bytes；data：Data
Output  : 0：Write successfully；1：Failed to write
**************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**************************************************************************
Function: IIC read register data
Input   : addr：Device address；reg：Register address；len;Number of bytes；*buf：Data read out
Output  : 0：Read successfully；1：Failed to read
**************************************************************************/

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**************************************************************************
Function: IIC reads a byte
Input   : ack：Send response signal or not；1：Send；0：Do not send
Output  : receive：Data read
**************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入 SDA is set as input
    for(i=0;i<8;i++ )
	 {
			IIC_SCL=0;
			delay_us(2);
			IIC_SCL=1;
			receive<<=1;
			if(READ_SDA)receive++;
			delay_us(2);
    }
    if (ack)
        IIC_Ack(); //发送ACK Send ACK
    else
        IIC_NAck();//发送nACK  Send nACK
    return receive;
}

/**************************************************************************
Function: IIC reads a byte
Input   : I2C_Addr：Device IIC address；addr:Register address
Output  : res：Data read
**************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;

	IIC_Start();
	IIC_Send_Byte(I2C_Addr);	   //发送写命令 Send write command
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //发送地址 Send Address
	IIC_Wait_Ack();
	//IIC_Stop();//产生一个停止条件 Generates a stop condition
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式 Entering receive mode
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);
    IIC_Stop();//产生一个停止条件 Generates a stop condition

	return res;
}

/**************************************************************************
Function: IIC continuous reading data
Input   : dev：Target device IIC address；reg:Register address；
					length：Number of bytes；*data:The pointer where the read data will be stored
Output  : count：Number of bytes read out-1
**************************************************************************/
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;

	IIC_Start();
	IIC_Send_Byte(dev);	   // Send write command
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   // Send Address
    IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(dev+1);  // Entering receive mode
	IIC_Wait_Ack();

    for(count=0;count<length;count++){

		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  // Read data with ACK
		 else                  data[count]=IIC_Read_Byte(0);  // Last byte NACK
	}
    IIC_Stop();//产生一个停止条件 Generates a stop condition
    return count;
}
/**************************************************************************
Function: Writes multiple bytes to the specified register of the specified device
Input   : dev：Target device IIC address；reg：Register address；length：Number of bytes；
					*data：The pointer where the read data will be stored
Output  : 1
**************************************************************************/
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){

 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   // Send write command
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   // Send Address
  IIC_Wait_Ack();
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	 }
	IIC_Stop();// Generates a stop condition

    return 1; //status == 0;
}

/**************************************************************************
Function: Reads a byte of the specified register of the specified device
Input   : dev：Target device IIC address；reg：Register address；*data：The pointer where the read data will be stored
Output  : 1
**************************************************************************/
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************************************************************
Function: Write a byte to the specified register of the specified device
Input   : dev：Target device IIC address；reg：Register address；data：Data to be writtenwill be stored
Output  : 1
**************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************************************************************
Function: Read, modify, and write multiple bits in a byte of the specified device specified register
Input   : dev：Target device IIC address；reg：Register address；length：Number of bytes；
					bitStart：Start bit of target byte；data：Stores the value of the target byte bit to be changed
**************************************************************************/
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}


/**************************************************************************
Function: Read, modify, and write one bit in a byte of the specified device specified register
Input   : dev：Target device IIC address；reg：Register address；
					bitNum：To modify the bitnum bit of the target byte；data：When it is 0, the target bit will be cleared, otherwise it will be set
Output  : 1：success；0：fail
**************************************************************************/
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}


