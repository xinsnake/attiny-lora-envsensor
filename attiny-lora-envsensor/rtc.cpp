/*
 * rtc.cpp
 *
 * Created: 17/01/2020 6:25:44 PM
 *  Author: xinsnake
 */
#include "rtc.h"

void RTC_init(void)
{
	while (RTC.STATUS > 0) {}				// Wait for all register to be synchronized
	RTC.PERH = 0xF0;						// triggers every 61440 cycles
	RTC.PERL = 0x00;						// = 61440 / 1.024kHz = 60 seconds
	RTC.INTCTRL = 0 << RTC_CMP_bp
				| 1 << RTC_OVF_bp;			// Overflow interrupt.
	RTC.CTRLA = RTC_PRESCALER_DIV1_gc		// NO Prescaler
				| 1 << RTC_RTCEN_bp       	// Enable RTC
				| 1 << RTC_RUNSTDBY_bp;   	// Run in standby
	RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;		// 32KHz divided by 32, i.e run at 1.024kHz
}