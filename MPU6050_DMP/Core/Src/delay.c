/*
 * delay.c
 *
 *  Created on: Dec 16, 2025
 *      Author: julie
 */


#include "delay.h"

uint32_t dwt_us;


HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
	DEM_CR |= 1<<24;

	DWT_CYCCNT = (uint32_t)0u;

	//3.使能CYCCNT，开启计时  Enable  CYCCNT and start timing
	DWT_CTRL |= 1<<0;

	//4.计算DWT微秒延迟函数的延迟因子  Calculate the delay factor of the DWT microsecond delay function
	dwt_us = HAL_RCC_GetSysClockFreq()/1000000;

	return HAL_OK;
}

//重写HAL_GetTick() Override HAL_GetTick()
uint32_t HAL_GetTick(void)
{
	//把计数值除以内核频率的1000分之一倍，实现1毫秒返回1
	//以C8T6为例：因为计数到72是1微秒（1÷72MHz）
	//能返回的最大的毫秒值是 2^32 - 1 / 72000 = 59652  (取整数)
	// Divide the count value by 1/1000 of the core frequency to return 1 in 1 millisecond
	// Take C8T6 as an example: because counting to 72 is 1 microsecond (1÷72MHz)
	// The maximum millisecond value that can be returned is 2^32 - 1 / 72000 = 59652 (rounded)
	return ((uint32_t)DWT_CYCCNT/(HAL_RCC_GetSysClockFreq()/1000));
}

//重写HAL_Delay()
//Delay 范围： 0 ~ ( 2^32-1 / （系统时钟÷1000））
//72MHz时钟： 0~59652
//84MHz时钟： 0~51130
//180MHz时钟：0~23860
//400Mhz时钟：0~10737
//Rewrite HAL_Delay()
//Delay range: 0 ~ (2^32-1 / (system clock ÷ 1000))
//72MHz clock: 0 ~ 59652
//84MHz clock: 0 ~ 51130
//180MHz clock: 0 ~ 23860
//400Mhz clock: 0 ~ 10737
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < __HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  //HAL_Delay()延迟函数重写后，需要考虑溢出的问题，原函数注释  After the HAL_Delay() delay function is rewritten, the overflow problem needs to be considered. The original function comments
//  while ((HAL_GetTick() - tickstart) < wait)
//  {
//  }

  wait += tickstart;   	   																 //计算所需要的计时时间  Calculate the time required
  if(wait>__HAL_MAX_DELAY) wait = wait - __HAL_MAX_DELAY;  //大于最大计数值则溢出,计算溢出部分  If it is greater than the maximum count value, it overflows and calculates the overflow part

  //计数没有溢出，直接等待到延迟时间即可 If the count does not overflow, just wait until the delay time.
   if(wait>tickstart)
  {
	while(HAL_GetTick()<wait);
  }
  //计数溢出 Count overflow
  else
  {
	while(HAL_GetTick()>wait); //未溢出部分计时 Timing of the non-overflow part
	while(HAL_GetTick()<wait); //溢出部分计时 Overflow Part Timing
  }
}

//返回32位计数器CYCCNT的值 Returns the value of the 32-bit counter CYCCNT
uint32_t DWT_CNT_GET(void)
{
	return((uint32_t)DWT_CYCCNT);
}

//使用DWT提供微秒级延迟函数 Using DWT to provide microsecond delay function
void HAL_Delay_us(uint32_t us)
{
	uint32_t BeginTime,EndTime,delaytime;

	//获取当前时间戳 Get the current timestamp
	BeginTime = DWT_CNT_GET();

	//计算需要延迟多少微秒 Calculate how many microseconds to delay
	delaytime = us*dwt_us;

	//起始+需要延迟的微秒 = 需要等待的时间  Start + microseconds of delay = time to wait
	EndTime = BeginTime+delaytime;

	//计数没有溢出，直接等待到延迟时间即可  If the count does not overflow, just wait until the delay time.
	if(EndTime>BeginTime)
	{
		while(DWT_CNT_GET()<EndTime);
	}

	//计数溢出 Count overflow
	else
	{
		while(DWT_CNT_GET()>EndTime); //未溢出部分计时  Timing of the non-overflow part
		while(DWT_CNT_GET()<EndTime); //溢出部分计时  Overflow Part Timing
	}
}

////使用DWT提供微秒级延迟函数,方法2 Using DWT to provide microsecond delay function, method 2
//void HAL_Delay_us(uint32_t us)
//{
//	//用于保存上一次计数值和当前计数值 Used to save the last count value and the current count value
//	uint32_t last_count,count,WaitTime;
//
//	//计算需要延迟多少个1微秒 Calculate how many 1 microseconds of delay are needed
//	uint32_t delaytime = us*dwt_us;
//
//	last_count = DWT_CNT_GET(); //获取当前的计数值 Get the current count value
//	WaitTime = 0;
//
//	//循环等待延迟时间 Loop wait delay time
//	while(WaitTime<delaytime)
//	{
//		count = DWT_CNT_GET();
//		//计时未溢出 The timer has not overflowed
//		if(count>last_count) WaitTime = count-last_count;
//
//		//计时溢出，加上32位的偏差值 Timing overflow, plus 32-bit deviation value
//		else				 WaitTime = count+0xffffffff - last_count;
//	}
//}


//======================= SysTick被释放，以下是对SysTick重新配置利用 SysTick is released, the following is the SysTick reconfiguration exploit ===========================//

u16 i_us;  //微秒因子 Microsecond Factor
u16 i_ms;  //毫秒因子 Millisecond Factor

//SysTick被释放后，使用SysTick编写延迟函数 After SysTick is released, use SysTick to write a delay function
void delay_init(void)
{
	SysTick->CTRL &= ~(1<<2); 				   //设定Systick时钟源，HCLK/8 Set Systick clock source, HCLK/8
	SysTick->CTRL &= ~(1<<1);				     //关闭由HAL库自带的SysTick中断，减少系统资源浪费 Disable the SysTick interrupt provided by the HAL library to reduce the waste of system resources
	i_us = HAL_RCC_GetSysClockFreq()/8000000;  //计算微秒因子 HCLK/晶振 Calculate the microsecond factor HCLK/crystal
	i_ms = i_us * 1000 ; 					   //计算毫秒因子 Calculating millisecond factors
}

// i_us * us 的值不可超过 2^24 -1 = 16777215
// 以72MHz的F103C8T6为例，i_us = 9 , us = 0~1,864,135
// The value of i_us * us cannot exceed 2^24 -1 = 16777215
// Taking the 72MHz F103C8T6 as an example, i_us = 9, us = 0~1,864,135
void delay_us(u32 us)
{
//	u32 temp;
//	SysTick -> LOAD = i_us * us; //计算需要设定的自动重装值 Calculate the auto-reload value that needs to be set
//	SysTick -> VAL = 0 ;         //清空计数器 Clear counter
//	SysTick -> CTRL |= 1<<0 ;    //开启计时 Start timing
//	do{
//		temp = SysTick -> CTRL; 		  			//读取CTRL寄存器的状态位，目的是获取第0位和最高位 Read the status bits of the CTRL register to get the 0th and highest bits
//	}while((temp&0x01)&&!(temp&(1<<16))); //如果计数器被使能且计时时间未到
//	SysTick->CTRL &= ~(1<<0);  			  		//计时结束，关闭倒数 Timer ends, close countdown
//	SysTick -> VAL = 0 ;       		      	//清空计数器 Clear counter
}

// i_ms * ms 的值不可超过 2^24 -1 = 16777215
// 以72MHz的F103C8T6为例，i_ms = 9000 , us = 0 ~ 1864
// The value of i_ms * ms cannot exceed 2^24 -1 = 16777215
// Taking the 72MHz F103C8T6 as an example, i_ms = 9000, us = 0 ~ 1864
void delay_ms(u16 ms)
{
	u32 temp;
	SysTick -> LOAD = i_ms * ms; //计算需要设定的自动重装值 Calculate the auto-reload value that needs to be set
	SysTick -> VAL = 0 ;         //清空计数器 Clear counter
	SysTick -> CTRL |= 1<<0 ;    //开启计时 Start timing
	do{
		temp = SysTick -> CTRL; 		  			//读取CTRL寄存器的状态位，目的是获取第0位和最高位 Read the status bits of the CTRL register to get the 0th and highest bits
	}while((temp&0x01)&&!(temp&(1<<16))); //如果计数器在使能，且倒数未到，则循环 If the counter is enabled and the countdown has not yet arrived, the loop
	SysTick->CTRL &= ~(1<<0);  			  		//计时结束，关闭倒数 Timer ends, close countdown
	SysTick -> VAL = 0 ;       		      	//清空计数器 Clear counter
}
