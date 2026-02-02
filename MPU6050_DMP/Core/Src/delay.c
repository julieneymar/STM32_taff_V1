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

	//3.  Enable  CYCCNT and start timing
	DWT_CTRL |= 1<<0;

	//4.  Calculate the delay factor of the DWT microsecond delay function
	dwt_us = HAL_RCC_GetSysClockFreq()/1000000;

	return HAL_OK;
}

//HAL_GetTick() Override HAL_GetTick()
uint32_t HAL_GetTick(void)
{

	// Divide the count value by 1/1000 of the core frequency to return 1 in 1 millisecond
	// Take C8T6 as an example: because counting to 72 is 1 microsecond (1÷72MHz)
	// The maximum millisecond value that can be returned is 2^32 - 1 / 72000 = 59652 (rounded)
	return ((uint32_t)DWT_CYCCNT/(HAL_RCC_GetSysClockFreq()/1000));
}


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


  wait += tickstart;   	   																 //  Calculate the time required
  if(wait>__HAL_MAX_DELAY) wait = wait - __HAL_MAX_DELAY;  //  If it is greater than the maximum count value, it overflows and calculates the overflow part

  // If the count does not overflow, just wait until the delay time.
   if(wait>tickstart)
  {
	while(HAL_GetTick()<wait);
  }
  // Count overflow
  else
  {
	while(HAL_GetTick()>wait); // Timing of the non-overflow part
	while(HAL_GetTick()<wait); // Overflow Part Timing
  }
}

// Returns the value of the 32-bit counter CYCCNT
uint32_t DWT_CNT_GET(void)
{
	return((uint32_t)DWT_CYCCNT);
}

// Using DWT to provide microsecond delay function
void HAL_Delay_us(uint32_t us)
{
	uint32_t BeginTime,EndTime,delaytime;

	// Get the current timestamp
	BeginTime = DWT_CNT_GET();

	// Calculate how many microseconds to delay
	delaytime = us*dwt_us;

	// Start + microseconds of delay = time to wait
	EndTime = BeginTime+delaytime;

	//  If the count does not overflow, just wait until the delay time.
	if(EndTime>BeginTime)
	{
		while(DWT_CNT_GET()<EndTime);
	}

	// Count overflow
	else
	{
		while(DWT_CNT_GET()>EndTime); //  Timing of the non-overflow part
		while(DWT_CNT_GET()<EndTime); //  Overflow Part Timing
	}
}


u16 i_us;  // Microsecond Factor
u16 i_ms;  // Millisecond Factor

// After SysTick is released, use SysTick to write a delay function
void delay_init(void)
{
	SysTick->CTRL &= ~(1<<2); 				   // HCLK/8 Set Systick clock source, HCLK/8
	SysTick->CTRL &= ~(1<<1);				     //  Disable the SysTick interrupt provided by the HAL library to reduce the waste of system resources
	i_us = HAL_RCC_GetSysClockFreq()/8000000;  // Calculate the microsecond factor HCLK/crystal
	i_ms = i_us * 1000 ; 					   // Calculating millisecond factors
}

static uint32_t dwt_us_factor;

void delay_us(u32 us)
{
	 uint32_t start = DWT->CYCCNT;
	 uint32_t ticks = us * dwt_us_factor;

	 while ((DWT->CYCCNT - start) < ticks);

}

void delay_ms(u16 ms)
{
	u32 temp;
	SysTick -> LOAD = i_ms * ms; //Calculate the auto-reload value that needs to be set
	SysTick -> VAL = 0 ;         // Clear counter
	SysTick -> CTRL |= 1<<0 ;    // Start timing
	do{
		temp = SysTick -> CTRL; 		  			// Read the status bits of the CTRL register to get the 0th and highest bits
	}while((temp&0x01)&&!(temp&(1<<16))); // If the counter is enabled and the countdown has not yet arrived, the loop
	SysTick->CTRL &= ~(1<<0);  			  		// Timer ends, close countdown
	SysTick -> VAL = 0 ;       		      	// Clear counter
}
