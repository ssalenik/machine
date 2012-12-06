#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
//#include <stdlib.h>
#include <math.h>

#define F_CPU 20000000
#define nop()	__asm__ __volatile__("nop")


/*! FUNCTION PROTOTYPES OF INCLUDED FILES */

// "public" functions of "delay.c"
void delay_us(uint16_t ticks_us);
void delay_ms(uint32_t ticks_ms);

// "public" functions of "hex.c"
uint8_t htoa(uint8_t MSB, uint8_t LSB);
void atoh(uint8_t data, uint8_t* array);
uint8_t isHex(uint8_t ascii);

// "public" functions of "ports.c"
void init_ports(void);

// "public" functions of "uart.c"
void init_uart(void);
uint8_t uart_available(uint8_t port);
uint8_t uart_get(uint8_t port);
void uart_put(uint8_t port, uint8_t c);
// NOTE: port is defined as DRIVE (0) or DEBUG (1)
// to print via uart, use: fprintf_P(&{drive|debug}, PSTR("format_string"), vars...);

// "public" functions of "adc.c"
void init_adc(void);
uint16_t read_adc(uint8_t channel);

// "public" functions of "timer.c"
void init_timer(void);
uint32_t uptime(void);
void set_speed_3(uint16_t speed);
void set_speed_4(uint16_t speed);
void servo5(uint8_t degrees);
void servo6(uint8_t degrees);
void servo7(uint8_t degrees);
void servo8(uint8_t degrees);

/*! END FUNCTION PROTOTYPES OF INCLUDED FILES */


#include "pt.h"

#include "delay.c"
#include "hex.c"
#include "ports.c"
#include "uart.c"
#include "adc.c"
#include "timer.c"

// for use in "immediate-return" tasks:
#define set_interval(milliseconds) \
	static uint32_t next_call = (milliseconds); \
	if(uptime() < next_call) return; \
	next_call += (milliseconds)

// for use in the "main_flow" thread:
#define sleep(milliseconds) \
	pt_target = uptime() + (milliseconds); \
	PT_WAIT_WHILE(pt, uptime() < pt_target)

#define RX_LINE_SIZE FIFO_LENGTH
#define VBAT_FACTOR	0.0044336
#define VBAT_LOW	767
#define VBAT_NORM	789

static struct pt pt1; // thread 1: pt_main_flow
static struct pt pt2; // thread 2: test

// thread/task list:
static int pt_main_flow(struct pt *pt);
static int pt_test(struct pt *pt); // test
void check_drive_uart(void);
void check_debug_uart(void);
void check_print_stat(void);

// function prototypes:
void stop();
void forward(uint8_t speed);
void backwards(uint8_t speed);
void freedrive(uint8_t speedL, uint8_t speedR, uint8_t dir);
void forward_dist(uint8_t speed, uint16_t dist);
void backwards_dist(uint8_t speed, uint16_t dist);

uint8_t run = 0;
uint8_t run_test = 0; // test
uint8_t drive_complete = 1;
uint8_t rev_passthru = 1;

void init(void) {
	init_ports();
	
	delay_ms(T_MS * 200); // wait for capacitors to charge
	
	init_uart();
	init_adc();
	init_timer();
	
	sei(); // enable interrupts
}

int main(void) {
	init();
	
	PT_INIT(&pt1); // required to initialize a thread
	PT_INIT(&pt2); // test
	
	// cycling through threads/tasks
	while(1) {
		if(run) pt_main_flow(&pt1);
		if(run_test) pt_test(&pt2);
		check_drive_uart();
		check_debug_uart();
		check_print_stat();
	}
	
	return 0;
}

static int pt_test(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	
	static uint32_t pt_target; // for delays
	
	if(run_test == 1) {
		freedrive(0, 0, 0); sleep(100);
		stop(); sleep(100);
	}
	else if(run_test == 2) {
		freedrive(0, 0, 0); sleep(100);
		stop(); sleep(100);
	}
	
	run_test = 0; // stop test thread after execution
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required to denote the beginning of a thread
}

static int pt_main_flow(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	
	static uint32_t pt_target; // for delays
	
	fprintf_P(&drive, PSTR("1001\r")); // PID ON
	sleep(3000); // startup delay
	
	servo5(0);
	
	backwards_dist(25, 50); PT_WAIT_UNTIL(pt, drive_complete); // push the hook
	stop(); sleep(200);
	
	//stop EVERYTHING. END OF EXECUTION
	run = 0;
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required as a denote the end of a thread
}

void stop()                                        { fprintf_P(&drive, PSTR("30\r"));              }
void forward(uint8_t speed)                        { fprintf_P(&drive, PSTR("31%02x00\r"), speed); }
void backwards(uint8_t speed)                      { fprintf_P(&drive, PSTR("31%02x11\r"), speed); }
void freedrive(uint8_t speedL, uint8_t speedR, uint8_t dir) { fprintf_P(&drive, PSTR("34%02x%02x%02x\r"), speedL, speedR, dir); }
// commands below return acknowledgement: "D\r"
void forward_dist(uint8_t speed, uint16_t dist)    { drive_complete = 0; fprintf_P(&drive, PSTR("32%02x%04x\r"), speed, dist); }
void backwards_dist(uint8_t speed, uint16_t dist)  { drive_complete = 0; fprintf_P(&drive, PSTR("33%02x%04x\r"), speed, dist); }

void check_drive_uart(void) {
	static uint8_t inputbuf[RX_LINE_SIZE], inputptr = 0;
	uint8_t i, recv;
	
	while(uart_available(DRIVE)) {
		recv = uart_get(DRIVE);
		
		if(recv == '\r') {
			if(inputptr) {
				if(inputbuf[0] == '@') {
					// reverse command to MAIN MCU
					// TODO
				}
				else if(rev_passthru) {
					for(i = 0; i < inputptr; i++) { uart_put(DEBUG, inputbuf[i]); }
					fprintf(&debug, "\r\n");
				}
				
				inputptr = 0;
			}
		}
		else if(recv == '\n') {
			;
		}
		else {
			if(inputptr != RX_LINE_SIZE) {
				inputbuf[inputptr] = recv;
				inputptr++;
			}
		}
	}
}

void check_debug_uart(void) {
	static uint8_t inputbuf[RX_LINE_SIZE], inputptr = 0;
	uint8_t i, recv;
	
	while(uart_available(DEBUG)) {
		recv = uart_get(DEBUG);
		
		if(recv == '\r') {
			fprintf(&debug, "\r\n");
			
			if(inputptr) {
				switch(inputbuf[0]) {
					case '?': // print drive command list
					fprintf_P(&debug, PSTR("stop() ......................... | p30\r\n"));
					fprintf_P(&debug, PSTR("forward(speed) ................. | p31%%02x00\r\n"));
					fprintf_P(&debug, PSTR("backwards(speed) ............... | p31%%02x11\r\n"));
					fprintf_P(&debug, PSTR("forward_dist(speed, dist) ...... | p32%%02x%%04x\r\n"));
					fprintf_P(&debug, PSTR("backwards_dist(speed, dist) .... | p33%%02x%%04x\r\n"));
					fprintf_P(&debug, PSTR("freedrive(speedL, speedR, dir) . | p34%%02x%%02x%%02x\r\n"));
					break;
					
					case 'p': // passthrough to DRIVE MCU
					for(i = 1; i < inputptr; i++) { uart_put(DRIVE, inputbuf[i]); }
					uart_put(DRIVE, '\r');
					break;
					
					case '0': // reverse passthrough OFF
					rev_passthru = 0;
					break;
					
					case '1': // reverse passthrough ON
					rev_passthru = 1;
					break;
					
					case 's': // start/stop main thread
					run ^= 1;
					if(run) { fprintf_P(&debug, PSTR("Main thread started!\r\n")); }
					else    { fprintf_P(&debug, PSTR("Main thread stopped!\r\n")); stop(); }
					break;
					
					case 't': // start/stop test thread
					if(inputptr == 2) {
						run_test = htoa(0, inputbuf[1]);
						if(!run_test) { fprintf_P(&debug, PSTR("All test sequences stopped!\r\n")); stop(); }
						else { fprintf_P(&debug, PSTR("Started test sequence %u!\r\n"), run_test); }
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case ' ':
					stop();
					break;
					
					case 'u':
					fprintf_P(&debug, PSTR("%lu\r\n"), uptime());
					break;
					
					case 'b':
					fprintf_P(&debug, PSTR("4 x %1.2fV\r\n"), (float)read_adc(VSENS) * VBAT_FACTOR);
					break;
					
					case 'm': // servo power
					if(inputptr >= 2 && (inputbuf[1] == '0' || inputbuf[1] == '1')) {
						inputbuf[1] == '0' ? clr_bit(SPWR) : set_bit(SPWR);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '5': // servo5 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo5(htoa(inputbuf[1], inputbuf[2]));
						//OCR0A = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '6': // servo6 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo6(htoa(inputbuf[1], inputbuf[2]));
						//OCR0B = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '7': // servo7 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo7(htoa(inputbuf[1], inputbuf[2]));
						//OCR2A = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '8': // servo8 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo8(htoa(inputbuf[1], inputbuf[2]));
						//OCR2B = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					default:
					fprintf_P(&debug, PSTR("Command error\r\n"));
				}
				
				inputptr = 0;
			}
		}
		else if(recv == 0x7f) {
			if(!inputptr) { uart_put(DEBUG, '\a'); }
			else {
				fprintf(&debug, "\b\e[K");
				inputptr--;
			}
		}
		else {
			if(inputptr == RX_LINE_SIZE) { uart_put(DEBUG, '\a'); }
			else {
				uart_put(DEBUG, recv);
				inputbuf[inputptr] = recv;
				inputptr++;
			}
		}
	}
}

void check_print_stat(void) {
	set_interval(100);
	
	uint16_t vbat = read_adc(VSENS);
	if(vbat < VBAT_LOW) set_bit(LED1);
	else if(vbat > VBAT_NORM) clr_bit(LED1);
}

