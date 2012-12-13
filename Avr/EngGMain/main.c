#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define F_CPU 20000000
#define nop()	__asm__ __volatile__("nop")

#include "pt.h"

#include "delay.c"
#include "hex.c"
#include "ports.c"
#include "uart.c"
#include "adc.c"
#include "enc.c"
#include "timer.c"
#include "pid.c"

// for use in "immediate-return" tasks:
#define set_interval(milliseconds) \
	static uint32_t next_call = (milliseconds); \
	if(uptime() < next_call) return; \
	next_call += (milliseconds)

// for use in threads:
#define sleep(milliseconds) \
	pt_target = uptime() + (milliseconds); \
	PT_WAIT_WHILE(pt, uptime() < pt_target)

/*! END OF DRIVERS' SECTION */


#define RX_LINE_SIZE FIFO_LENGTH

uint8_t run_main = 0;
uint8_t run_test = 0; // test
uint8_t drive_complete = 1;
uint8_t rev_passthru = 1;
uint8_t local_dump = 1;

void stop()                                        { fprintf_P(&drive, PSTR("00\r"));              }
void forward(uint8_t speed)                        { fprintf_P(&drive, PSTR("31%02x00\r"), speed); }
void backwards(uint8_t speed)                      { fprintf_P(&drive, PSTR("31%02x11\r"), speed); }
void freedrive(uint8_t speedL, uint8_t speedR, uint8_t dir) { fprintf_P(&drive, PSTR("34%02x%02x%02x\r"), speedL, speedR, dir); }
// commands below return acknowledgement: "D\r"
void forward_dist(uint8_t speed, uint16_t dist)    { drive_complete = 0; fprintf_P(&drive, PSTR("32%02x%04x\r"), speed, dist); }
void backwards_dist(uint8_t speed, uint16_t dist)  { drive_complete = 0; fprintf_P(&drive, PSTR("33%02x%04x\r"), speed, dist); }

#include "pt-main.c"
#include "pt-test.c"
#include "rx-drive.c"
#include "rx-debug.c"
#include "tasks.c"

void init(void) {
	init_ports();
	
	init_uart();
	init_adc();
	init_enc();
	init_timer();
	
	sei(); // enable interrupts
	
	delay_ms(T_MS * 10); // wait for devices to initialize
	
	reset_pid();
}

int main(void) {
	init();
	
	static struct pt pt1; // thread 1: pt_main_flow
	static struct pt pt2; // thread 2: test
	
	PT_INIT(&pt1); // required to initialize a thread
	PT_INIT(&pt2); // test
	
	// cycling through threads/tasks
	while(1) {
		if(run_main) pt_main(&pt1);
		if(run_test) pt_test(&pt2);
		check_drive_uart();
		check_debug_uart();
		check_sys_state();
		update_pid_vals();
		check_print_stat();
	}
	
	return 0;
}

