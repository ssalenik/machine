/** -------------------------------------------------------------------------
 *      PORT CONNECTIONS
 *  -------------------------------------------------------------------------
 *   #	NAME	ALT	INT	I/O	PU	DEFS	FUNCTION
 *  -------------------------------------------------------------------------
 */

#define F_CPU 8000000
#define nop()	__asm__ __volatile__("nop")

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "sched.c"
#include "delay.c"

/**
 *  Scheduled functions go here
 */

int main(void) {
	
	/**
	 *  Initialization goes here (including function scheduling)
	 */
	
	rtc_init();
	
	return 0;
}

SIGNAL(TIMER0_COMPA_vect) {
	
	/**
	 *  Timer interrupt actions go here
	 */
	
	rtc_intr();
}

