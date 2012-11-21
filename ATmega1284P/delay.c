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

void delay_us(uint16_t ticks_us) {
	uint8_t i;
	uint8_t reps = ticks_us >> 8;
	uint8_t rmdr = ticks_us & 0xff;
	
	for(i = 0; i < reps; i++) _delay_loop_1(0);
	if(rmdr) _delay_loop_1(rmdr);
}

void delay_ms(uint32_t ticks_ms) {
	uint16_t i;
	uint16_t reps = ticks_ms >> 16;
	uint16_t rmdr = ticks_ms & 0xffff;
	
	for(i = 0; i < reps; i++) _delay_loop_2(0);
	if(rmdr) _delay_loop_2(rmdr);
}

