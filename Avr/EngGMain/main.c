#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define F_CPU 20000000
#define nop()	__asm__ __volatile__("nop")

#include "pt.h"

#include "../Common/common.h"
#include "track.c"

#include "delay.c"
#include "hex.c"
#include "ports.c"
#include "uart.c"
#include "timer.c"
#include "adc.c"
#include "enc.c"
#include "pid.c"

// for use in "immediate-return" tasks:
#define SET_INTERVAL(milliseconds) \
	static uint32_t next_call = (milliseconds); \
	if(uptime() < next_call) return; \
	next_call += (milliseconds)

// for use in threads:
#define SLEEP(milliseconds) \
	pt_target = uptime() + (milliseconds); \
	PT_WAIT_WHILE(pt, uptime() < pt_target)

/*! END OF DRIVERS' SECTION */


#define RX_LINE_SIZE FIFO_LENGTH
#define DRIVE_SPEED  200

uint8_t run_main = 0; // AUTOSTART
uint8_t run_test = 0; // test
uint8_t drive_complete = 1;
uint8_t rev_passthru = 1;
uint8_t local_dump = 1;
uint8_t sectL, sectR, trksL, trksR;
int16_t relpL, relpR, abspL, abspR;

// DRIVE MCU CONTROL FUNCTIONS
void stop(void)                                   { fprintf_P(&drive, PSTR("00\r")); }
void nav_stop(void)                               { fprintf_P(&drive, PSTR("30\r")); }
void fwd_both(uint8_t speed)                      { fprintf_P(&drive, PSTR("0300\r0400\r15%02x\r"), speed); }
void rev_both(uint8_t speed)                      { fprintf_P(&drive, PSTR("0301\r0401\r15%02x\r"), speed); }
void forward (uint8_t Lspeed, uint8_t Rspeed)     { fprintf_P(&drive, PSTR("0300\r0400\r11%02x\r12%02x\r"), Lspeed, Rspeed); }
void reverse (uint8_t Lspeed, uint8_t Rspeed)     { fprintf_P(&drive, PSTR("0301\r0401\r11%02x\r12%02x\r"), Lspeed, Rspeed); }
void turnCCW (uint8_t Lspeed, uint8_t Rspeed)     { fprintf_P(&drive, PSTR("0301\r0400\r11%02x\r12%02x\r"), Lspeed, Rspeed); }
void turnCW  (uint8_t Lspeed, uint8_t Rspeed)     { fprintf_P(&drive, PSTR("0300\r0401\r11%02x\r12%02x\r"), Lspeed, Rspeed); }
void set_abs_pos (int16_t pos)                    { fprintf_P(&drive, PSTR("1a%04x\r"), pos); }
void set_rel_pos (uint8_t sect, int16_t pos)      { fprintf_P(&drive, PSTR("1c%02x%04x\r"), sect, pos); }
void drv_pid_on  (void)                           { fprintf_P(&drive, PSTR("1001\r")); }
void drv_pid_off (void)                           { fprintf_P(&drive, PSTR("1000\r")); }
void pos_corr_on (void)                           { fprintf_P(&drive, PSTR("1f01\r")); }
void pos_corr_off(void)                           { fprintf_P(&drive, PSTR("1f00\r")); }
void rampdown_on (void)                           { fprintf_P(&drive, PSTR("3f01\r")); } // WARNING: power ramps up at fixed rate
void rampdown_off(void)                           { fprintf_P(&drive, PSTR("3f00\r")); } // WARNING: power drops to 0 immediately
void nav_forward (uint8_t Lspeed, uint8_t Rspeed) { fprintf_P(&drive, PSTR("33%02x%02x00\r"), Lspeed, Rspeed); }
void nav_reverse (uint8_t Lspeed, uint8_t Rspeed) { fprintf_P(&drive, PSTR("33%02x%02x11\r"), Lspeed, Rspeed); }
void nav_turnCCW (uint8_t Lspeed, uint8_t Rspeed) { fprintf_P(&drive, PSTR("33%02x%02x10\r"), Lspeed, Rspeed); }
void nav_turnCW  (uint8_t Lspeed, uint8_t Rspeed) { fprintf_P(&drive, PSTR("33%02x%02x01\r"), Lspeed, Rspeed); }
// commands below return acknowledgement: "@<cmd>\r"
void nav_abs_pos   (uint8_t speed, int16_t pos)               { drive_complete = 0; fprintf_P(&drive, PSTR("31%02x%04x\r"), speed, pos); }
void nav_rel_pos   (uint8_t speed, uint8_t sect, uint8_t pos) { drive_complete = 0; fprintf_P(&drive, PSTR("32%02x%02x%02x\r"), speed, sect, pos); }
void nav_dist      (uint8_t speed, int16_t dist)              { drive_complete = 0; fprintf_P(&drive, PSTR("34%02x%04x\r"), speed, dist); }
void nav_universal (uint8_t Lspeed, uint8_t Rspeed, uint8_t Lsect, uint8_t Rsect, int16_t Lpos, int16_t Rpos) {
	drive_complete = 0;
	fprintf_P(&drive, PSTR("35%02x%02x%02x%02x%04x%04x\r"), Lspeed, Rspeed, Lsect, Rsect, Lpos, Rpos);
}
// commands below return a reply, and therefore are preceded with '@'
void request_abs_pos(void)                        { fprintf_P(&drive, PSTR("@24\r")); }
void request_rel_pos(void)                        { fprintf_P(&drive, PSTR("@25\r")); }
void request_trks(void)                           { fprintf_P(&drive, PSTR("@26\r")); }

//  ARM  CONTROL FUNCTIONS: see file "pid.c"
// SERVO CONTROL FUNCTIONS: see file "timer.c"

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
	set_bit(SPWR); // power on servos
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
		update_pid_vals();
		check_sys_state();
		check_print_stat();
		check_track_sens();
	}
	
	return 0;
}

