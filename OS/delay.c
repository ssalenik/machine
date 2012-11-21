/**
 *  Software delay functions
 *  
 *  Usage:
 *  delay_us(T_US * time) where "time" is in us (integer)
 *  delay_ms(T_MS * time) where "time" is in ms (integer)
 *  
 *  WARNING: delays can be slightly longer than requested
 */

#define T_US	F_CPU / 3000000.0
#define T_MS	F_CPU / 4000.0

#include <util/delay_basic.h>

void delay_us(unsigned short ticks_us) {
	unsigned char i;
	unsigned char reps = ticks_us >> 8;
	unsigned char rmdr = ticks_us & 0xff;
	
	for(i = 0; i < reps; i++) _delay_loop_1(0);
	if(rmdr) _delay_loop_1(rmdr);
}

void delay_ms(unsigned long ticks_ms) {
	unsigned short i;
	unsigned short reps = ticks_ms >> 16;
	unsigned short rmdr = ticks_ms & 0xffff;
	
	for(i = 0; i < reps; i++) _delay_loop_2(0);
	if(rmdr) _delay_loop_2(rmdr);
}

