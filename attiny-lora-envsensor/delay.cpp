#include "delay.h"

void delay_ms(uint32_t ms)
{
	while (ms--)
	{
		_delay_ms(1);
	}
}

void delay_us(uint32_t us)
{
	while (us--)
	{
		_delay_us(1);
	}
}