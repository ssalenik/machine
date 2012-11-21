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
inline uint8_t IR_hole_L(void);
inline uint8_t IR_hole_R(void);
inline uint8_t IR_hole_F(void);
inline void IR_enable_L(void);
inline void IR_enable_R(void);
inline void IR_enable_F(void);
inline void IR_enable_dist(void);
inline void IR_disable_L(void);
inline void IR_disable_R(void);
inline void IR_disable_F(void);
inline void IR_disable_dist(void);
inline void enable_descend(void);
inline void disable_descend(void);
inline void EM_enable(void);
inline void EM_disable(void);
inline uint8_t EM_status(void);

// "public" functions of "uart.c"
void init_uart(void);
uint8_t uart_available(uint8_t port);
uint8_t uart_get(uint8_t port);
void uart_put(uint8_t port, uint8_t c);
// NOTE: port is defined as DRIVE (0) or DEBUG (1)
// to print via uart, use: fprintf_P(&{drive|debug}, PSTR("format_string"), vars...);

// "public" functions of "acc.c"
void init_acc(void);
void acc_update(void);
void read_acc_values(float *values);
void read_acc_angles(float *angles);

// "public" functions of "adc.c"
void init_adc(void);
uint16_t IR_read_dist(void);

// "public" functions of "timer.c"
void init_timer(void);
uint32_t uptime(void);
void servo1(uint8_t degrees);
void servo2(uint8_t degrees);

/*! END FUNCTION PROTOTYPES OF INCLUDED FILES */


#include "pt.h"

#include "delay.c"
#include "hex.c"
#include "ports.c"
#include "uart.c"
#include "acc.c"
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

// for use in the "test" thread:
#define sleep_test(milliseconds) \
	pt_target_test = uptime() + (milliseconds); \
	PT_WAIT_WHILE(pt, uptime() < pt_target_test)

#define RX_LINE_SIZE FIFO_LENGTH
#define SENSOR_DELAY 100
#define SERVO_MIN 42
#define SERVO_MAX 136
#define CLEARANCE_DELAY 400
#define IR_NUM_SAMPLES  300
#define IR_SAMP_INTERVAL 30
#define IR_NOISE_LEVEL  100
#define IR_THR_RATIO_UP 2/3
#define IR_THR_RATIO_DN 2/5

static struct pt pt1; // thread 1: pt_main_flow
static uint32_t pt_target;

static struct pt pt2; // thread 2: test
static uint32_t pt_target_test; // test

// thread/task list:
static int pt_main_flow(struct pt *pt);
static int pt_test(struct pt *pt); // test
void check_drive_uart(void);
void check_debug_uart(void);
void check_acc_update(void);
void check_print_stat(void);

// function prototypes:
void stop();
void forward(uint8_t speed);
void backwards(uint8_t speed);
void freedrive(uint8_t speedL, uint8_t speedR, uint8_t dir);
void turn_left(uint8_t speed);
void turn_right(uint8_t speed);
void forward_dist(uint8_t speed, uint16_t dist);
void backwards_dist(uint8_t speed, uint16_t dist);
void turn_ccw_by(uint16_t angle);
void turn_cw_by(uint16_t angle);
void orient_self(int16_t angle);
void orient_self_0();
void orient_self_90();
void orient_self_180();
void orient_self_270();
float get_heading();
float read_acc_mag2(void);
uint8_t is_falling(float fall_threshold);
uint8_t is_climbing(float climb_threshold);
void disable_rotadj();
void enable_rotadj();

// inter-task variables:
uint8_t settings[10][10] = {
	{0x32, 0x00, 0x96, 0x20, 0x18, 0x0a, 0x00, 0x00, 0x00, 0x00}, /* vertical distance, angleH, angleL, speedL, speedR, time */
	{0x30, 0x20, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* curve right: speedL, speedR, dir, duration */
	{0x20, 0x20, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* approach from left: speedL, speedR, dir, duration */
	{0x20, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* manouver right: speed, duration */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /*  */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /*  */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /*  */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /*  */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /*  */
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  /*  */
};

uint8_t run = 1;
uint8_t run_test = 0; // test
uint8_t drive_complete = 1;
uint8_t print_distance = 0;
uint8_t print_acc = 0;

void init(void) {
	init_ports();
	
	delay_ms(T_MS * 200); // wait for capacitors to charge
	
	init_uart();
	init_acc();
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
		check_acc_update();
		check_print_stat();
	}
	
	return 0;
}

static int pt_test(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	int heading_norm;
	
	if(run_test == 1) {
		// curve right: speedL, speedR, dir, duration
		freedrive(settings[1][0], settings[1][1], settings[1][2]);
		sleep_test(settings[1][3] * 100);
		stop();
	}
	else if(run_test == 2) {
		// approach from left: speedL, speedR, dir, duration
		freedrive(settings[2][0], settings[2][1], settings[2][2]);
		sleep_test(settings[2][3] * 100);
		stop();
	}
	else if(run_test == 3) {
		// manouver right: speed, duration
		freedrive(settings[3][0], 0, 0x11); sleep_test(settings[3][1] * 100);
		stop(); sleep_test(250);
		freedrive(0, settings[3][0], 0x11); sleep_test(settings[3][1] * 100);
		stop(); sleep_test(250);
		freedrive(settings[3][0], 0, 0x00); sleep_test(settings[3][1] * 100);
		stop(); sleep_test(250);
		freedrive(0, settings[3][0], 0x00); sleep_test(settings[3][1] * 100);
		stop(); sleep_test(250);
	}
	else if(run_test == 4) {
		// servo-motor approach
		
		// facing platform from right side
		
		servo1(75);
		sleep_test(500); // precise timing
		forward(32);
		PT_WAIT_UNTIL(pt, is_climbing(30));
		fprintf(&debug, "\a");
		sleep_test(1200);
		servo1(120);
		sleep_test(2000);
		stop();
		
		// got up the platform here
		sleep_test(500);
		servo1(SERVO_MIN);
		sleep_test(500);
		
		//freedrive(20, 0, 0x10); sleep_test(800); // precise timing
		//stop(); sleep_test(500);
		
		disable_rotadj();
		orient_self(settings[0][1] << 8 | settings[0][2]); PT_WAIT_UNTIL(pt, drive_complete);
		freedrive(settings[0][3], settings[0][4], 0x00);
		sleep_test(settings[0][5] * 100);
		stop();
	}
	else if(run_test == 5) {
		orient_self(10); PT_WAIT_UNTIL(pt, drive_complete); // turn right
		stop(); sleep_test(200);
		
		forward(50); PT_WAIT_UNTIL(pt, IR_read_dist() > 500); // go right FIXME: find the right value
		forward_dist(50, 50); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep_test(200);
		
		orient_self(94); PT_WAIT_UNTIL(pt, drive_complete); // turn left
		stop(); sleep_test(200);
		
		forward(50); PT_WAIT_UNTIL(pt, IR_read_dist() > 500); 
		forward_dist(50, 85); PT_WAIT_UNTIL(pt, drive_complete);// go up FIXME: find the right value
		stop(); sleep_test(200);
		
		IR_disable_dist(); sleep_test(SENSOR_DELAY);
		
		orient_self(179); PT_WAIT_UNTIL(pt, drive_complete); // turn left
		stop(); sleep_test(200);
		
		// facing platform from right side
		servo1(75); sleep_test(500);
		forward(32); PT_WAIT_UNTIL(pt, is_climbing(30.0));
		fprintf(&debug, "\a");
		
		//FIXME: this value must be calibrated
		//delay after the root passes the 30 degree treshold before it deploys its rear servo
		sleep_test(1000); //original value: 1000
		servo1(120);
		sleep_test(1250); 
		stop(); sleep_test(1000);
		
		// got up the platform here
		servo1(SERVO_MIN); sleep_test(500);
		
		forward(32); sleep_test(3000); //FIXME: find right value so the ball is pushed
		stop(); sleep_test(100);
	}
	else {
		// after bridge was crossed
		
		IR_enable_dist(); sleep_test(SENSOR_DELAY);
		orient_self(20); PT_WAIT_UNTIL(pt, drive_complete); // turn right
		stop(); sleep_test(200);
		
		forward(50); PT_WAIT_UNTIL(pt, IR_read_dist() > 500); // go right FIXME: find the right value
		forward_dist(50, 50); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep_test(500);
		
		orient_self(90); PT_WAIT_UNTIL(pt, drive_complete); // turn left
		stop(); sleep_test(500);
		
		forward(50); PT_WAIT_UNTIL(pt, IR_read_dist() > 500); // go up FIXME: find the right value
		forward_dist(50, 85); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep_test(200);
		
		IR_disable_dist(); sleep_test(SENSOR_DELAY);
		
		orient_self(180); PT_WAIT_UNTIL(pt, drive_complete); // turn left
		stop(); sleep_test(200);
		
		// facing platform from right side
		servo1(75); sleep_test(500); // precise timing
		forward(32); PT_WAIT_UNTIL(pt, is_climbing(30.0));
		fprintf(&debug, "\a");
		
		//FIXME: this value must be calibrated
		//delay after the root passes the 30 degree treshold before it deploys its rear servo
		sleep_test(1000); //original value: 1000
		servo1(120);
		sleep_test(1500);
		stop(); sleep_test(1000);
		
		// got up the platform here
		servo1(SERVO_MIN); sleep_test(500);
		
		//freedrive(20, 0, 0x10); sleep_test(800); // precise timing
		//stop(); sleep_test(500);
		
		disable_rotadj();
		orient_self(settings[0][1] << 8 | settings[0][2]); PT_WAIT_UNTIL(pt, drive_complete);
		freedrive(settings[0][3], settings[0][4], 0x00);
		sleep_test(settings[0][5] * 100);
		stop();
		
		// BACKUP PLAN TERMINATE HERE
		
		// forward_dist(50, 250); PT_WAIT_UNTIL(pt, drive_complete); // grab the ball
		// sleep_test(500);
		// disable_rotadj();
		// orient_self(180); PT_WAIT_UNTIL(pt, drive_complete);
		// stop(); sleep_test(1000);
		// enable_rotadj();
		// // get off the platform
		// forward_dist(70, 250); PT_WAIT_UNTIL(pt, drive_complete);
		// sleep_test(500);
		// backwards_dist(50, 30); PT_WAIT_UNTIL(pt, drive_complete);
		// sleep_test(500);
		// orient_self(160); PT_WAIT_UNTIL(pt, drive_complete);
		
		// // get to ledge
		// IR_enable_F(); sleep_test(SENSOR_DELAY);
		// forward(25); PT_WAIT_UNTIL(pt, IR_hole_F());
		// stop(); sleep_test(500);
		// IR_disable_F(); sleep_test(SENSOR_DELAY);
		
		// // drop-off the mutherfucking ball
		// /*IR_enable_R(); sleep_test(SENSOR_DELAY);
		// freedrive(0, 50, 0x00); PT_WAIT_UNTIL(pt, IR_hole_R());
		// stop(); sleep_test(500);
		// IR_disable_R(); sleep_test(SENSOR_DELAY);*/
		// freedrive(0, 50, 0x00);
		// sleep_test(500);
		// do {
			// sleep_test(50);
			// heading_norm = get_heading();
			// if (heading_norm < 0) heading_norm += 360;
		// } while (heading_norm < 230);
		// stop();
		// sleep_test(500);
		// orient_self(230);
		// stop();
		
		// // bitches that's it!
	}
	
	enable_rotadj();
	run_test = 0; // stop test thread after execution
	//PT_WAIT_WHILE(pt, 1);
	PT_END(pt); // required to denote the beginning of a thread
}

static int pt_main_flow(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	float acc_a[3];
	float heading, mag2;
	static float last_mag2 = 0;
	static uint32_t t_UP, t_DN, t_pass;
	static uint16_t i, plank, plank_max, plank_THR_UP, plank_THR_DN;
	
	fprintf_P(&drive, PSTR("1001\r")); // PID ON
	sleep(3000); // startup delay
	
	servo1(SERVO_MIN);
	
	backwards_dist(25, 50); PT_WAIT_UNTIL(pt, drive_complete); // push the hook
	stop(); sleep(200);
	
	// START AUTOMATIC
	// forward(25);
	// PT_WAIT_UNTIL(pt, is_falling(60.0)); // advance
	// EM_enable();
	// i2c_wr(ACC_ADDR, 0x23, 0xb0); // FS = 8g
	// print_acc = 1; // test: start printing accelerometer values @ 50 Hz
	// PT_WAIT_UNTIL(pt, is_falling(45.0));
	// stop();
	// PT_WAIT_UNTIL(pt, read_acc_mag2() > 0.14); // (1.5g / 4)^2
	
	// mag2 = read_acc_mag2();
	// do {
		// last_mag2 = mag2;
		// sleep(20);
		// mag2 = read_acc_mag2();
	// } while(mag2 >= last_mag2);
	
	// EM_disable();
	// print_acc = 0; // test: stop printing accelerometer values
	// i2c_wr(ACC_ADDR, 0x23, 0x80); // FS = 2g
	// sleep(3000); // drop and land
	// STOP AUTOMATIC
	
	// START MANUAL
	forward(40); 
	sleep(250); 
	forward(70); 
	PT_WAIT_UNTIL(pt, is_falling(60.0)); // advance
	forward(40); 
	EM_enable(); sleep(1000); // robot is falling
	stop(); sleep(10500); // robot is hanging and stabilizing
	EM_disable(); sleep(2000); // drop and land
	// STOP MANUAL
	
	// forward(65); sleep(500); // push to fall
	// stop(); sleep(3000); // the robot falls here
	
	read_acc_angles(acc_a);
	if(acc_a[2] < 65) {
		backwards(25); sleep(2000);
		stop(); sleep(250);
	}
	
	orient_self(105); PT_WAIT_UNTIL(pt, drive_complete); // to recover
	stop(); sleep(200);
	
	IR_enable_F(); //sleep(SENSOR_DELAY);
	
	forward(40); sleep(350); 
	forward(70); PT_WAIT_UNTIL(pt, IR_hole_F()); // go up
	stop(); IR_disable_F(); sleep(200);
	
	//orient_self(75); PT_WAIT_UNTIL(pt, drive_complete);
	//stop(); sleep(200);
	
	//backwards_dist(25, 40); PT_WAIT_UNTIL(pt, drive_complete); // adjustment down
	//stop(); sleep(200);
	
	//FIXME: this value must be calibrated
	//heading left after the fall
	orient_self(176); PT_WAIT_UNTIL(pt, drive_complete); // turn left | ORIG=180
	stop(); sleep(200);
	
	IR_enable_dist(); IR_enable_R(); //sleep(SENSOR_DELAY);
	
	forward(40); sleep(250);
	forward(100); // go left
	while(IR_read_dist() < 550)
	{
	    // advance
	    if(IR_hole_R())
	    {
		// if there's a hole on the right (close), we move to the left
		stop(); sleep(200);
		turn_ccw_by(30); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep(200);
		forward(50); sleep(1000);
		stop(); sleep(200);
		orient_self(176); PT_WAIT_UNTIL(pt, drive_complete);
		forward(40); sleep(350); forward(100);
	    }
	    else sleep(50); // needed to avoid conditional blocking
	}
	forward_dist(50, 40); PT_WAIT_UNTIL(pt, drive_complete);
	stop(); sleep(200);
	
	IR_disable_dist(); IR_disable_R(); //sleep(SENSOR_DELAY);
	
	//FIXME: this value must be calibrated
	//going up for the first time
	orient_self(84); PT_WAIT_UNTIL(pt, drive_complete); // turn right | ORIG=86
	stop(); sleep(200);
	
	IR_enable_F(); IR_enable_R(); //sleep(SENSOR_DELAY);
	
	forward(40); sleep(350); 
	forward(80); PT_WAIT_UNTIL(pt, IR_hole_F() || IR_hole_R());
	stop(); sleep(200);
	
	if(IR_hole_R()) {
		orient_self(90); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep(200);
	}
	
	if(!IR_hole_F()) {
		forward(40); sleep(250); 
		forward(80); PT_WAIT_UNTIL(pt, IR_hole_F());
		stop(); sleep(200);
	}
	
	IR_disable_F(); IR_disable_R(); //sleep(SENSOR_DELAY);
	
	/*
	IR_enable_F(); IR_enable_R(); sleep(SENSOR_DELAY);
	
	forward(50); // go up
	while(!IR_hole_F())
	{
	    // advance
	    if(IR_hole_R())
	    {
		// if there's a hole on the right (close), we move to the left
		stop(); sleep(1000);
		turn_ccw_by(30); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep(1000);
		forward(50); sleep(1000);
		stop(); sleep(1000);
		orient_self(90); PT_WAIT_UNTIL(pt, drive_complete);
		forward(50);
	    }
	    else sleep(50); // needed to avoid conditional blocking
	}
	stop(); sleep(1000);
	
	IR_disable_F(); IR_disable_R(); sleep(SENSOR_DELAY);
	
	orient_self(90); PT_WAIT_UNTIL(pt, drive_complete); // adjust direction
	stop(); sleep(1000);
	*/
	
	forward_dist(25, 50); PT_WAIT_UNTIL(pt, drive_complete); // adjustment up
	stop(); sleep(200);
	
	//FIXME: looks good, but doule-check
	//driving backwards towards the right
	orient_self(-165); PT_WAIT_UNTIL(pt, drive_complete); // turn left | ORIG=195
	stop(); sleep(200);
	
	/*
	IR_enable_R(); IR_enable_L(); IR_enable_dist(); sleep(SENSOR_DELAY);
	
	backwards(50); // go right
	while(IR_read_dist() > 150) //FIXME: find right value
	{
	    // advance backwards
	    if(IR_hole_R())
	    {
		// if there's a hole on the left (close), we move to the right
		stop(); sleep(1000);
		turn_cw_by(30); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep(1000);
		backwards(50); sleep(1000);
		stop(); sleep(1000);
		orient_self(180); PT_WAIT_UNTIL(pt, drive_complete);
		backwards(50);
	    }
	    else if(IR_hole_L())
	    {
		// if there's a hole on the right (close), we move to the left
		stop(); sleep(1000);
		turn_ccw_by(30); PT_WAIT_UNTIL(pt, drive_complete);
		stop(); sleep(1000);
		backwards(50); sleep(1000);
		stop(); sleep(1000);
		orient_self(180); PT_WAIT_UNTIL(pt, drive_complete);
		backwards(50);
	    }
	    else sleep(50); // needed to avoid conditional blocking
	}
	stop(); sleep(1000);
	
	IR_disable_R(); IR_disable_L(); IR_disable_dist(); sleep(SENSOR_DELAY);
	*/
	
	IR_enable_dist(); //sleep(SENSOR_DELAY);
	
	backwards(40); sleep(250); 
	backwards(80); PT_WAIT_UNTIL(pt, IR_read_dist() < 200); // go right
	backwards_dist(40, 100); PT_WAIT_UNTIL(pt, drive_complete);
	stop(); sleep(200);
	
	IR_disable_dist(); //sleep(SENSOR_DELAY);
	
	//FIXME: this value must be calibrated
	//approach towards the tunnel
	orient_self(88); PT_WAIT_UNTIL(pt, drive_complete); // turn right | ORIG=90
	stop(); sleep(200);
	
	forward_dist(60, 450); PT_WAIT_UNTIL(pt, drive_complete); // go up
	stop(); sleep(500);
	
	heading = get_heading(); // re-adjust with correction
	//re-calibration half-way on the approach to the tunnel
	if(fabs(heading - 89) > 3) {//FIXME: this value must be calibrated
		orient_self((int16_t)(178 - heading)); PT_WAIT_UNTIL(pt, drive_complete); // orig. value = 180
		// 90 - (heading - 90) => 85 - (heading - 85) => ANGLE ON PREV. LINE = 2 X TARGET HEADING!
		stop(); sleep(200);
	}
	
	forward_dist(60, 250); PT_WAIT_UNTIL(pt, drive_complete); // go up
	stop(); sleep(500);
	
	// heading = get_heading(); // re-adjust with correction
	// //re-calibration right before the first blade
	// if(fabs(heading - 87) > 3) {//FIXME: this value must be calibrated
		// orient_self((int16_t)(174 - heading)); PT_WAIT_UNTIL(pt, drive_complete);
		// // 90 - (heading - 90) => ANGLE ON PREV. LINE = 2 X TARGET HEADING!
		// stop(); sleep(200);
	// }
	
	/* THE BRIDGE! */
	
	IR_enable_dist(); sleep(SENSOR_DELAY);
	
	/* PLANK DETECTION ALGORITHM */
	
	// // find maximum reading over 0.75 plank revolutions
	// plank_max = 0;
	// for(i = 0; i < IR_NUM_SAMPLES; i++) {
		// plank = IR_read_dist();
		// if(plank > plank_max) { plank_max = plank; }
		// sleep(IR_SAMP_INTERVAL);
	// }
	// fprintf_P(&debug, PSTR("Peak IR = %u\r\n"), plank_max);
	
	// calc. thresholds
	plank_THR_UP = 400;
	plank_THR_DN = 280;
	
	// wait for negative transition
	//PT_WAIT_UNTIL(pt, IR_read_dist() > plank_THR_UP);
	PT_WAIT_UNTIL(pt, IR_read_dist() < plank_THR_DN);
	
	// get center time and pass a fixed time after
	PT_WAIT_UNTIL(pt, IR_read_dist() > plank_THR_UP); t_UP = uptime();
	PT_WAIT_UNTIL(pt, IR_read_dist() < plank_THR_DN); t_DN = uptime();
	t_pass = t_DN - ((t_DN - t_UP) >> 1) + CLEARANCE_DELAY;
	if(t_pass < t_DN) { t_pass = t_DN; }
	PT_WAIT_WHILE(pt, uptime() < t_pass);
	
	/* END PLANK DETECTION ALGORITHM */
	
	forward_dist(120, 400); PT_WAIT_UNTIL(pt, drive_complete); // go past the 1st plank
	stop(); sleep(400);
	
	IR_disable_dist(); IR_enable_L(); IR_enable_R(); sleep(SENSOR_DELAY);
	
	//re-calibration before the second blade
	if      (IR_hole_L())                  { orient_self(81); PT_WAIT_UNTIL(pt, drive_complete); }
	else if (IR_hole_R())                  { orient_self(93); PT_WAIT_UNTIL(pt, drive_complete); }
	else if (fabs(get_heading() - 87) > 3) { orient_self(87); PT_WAIT_UNTIL(pt, drive_complete); }
	stop(); sleep(200);
	
	IR_disable_L(); IR_disable_R(); IR_enable_dist(); sleep(SENSOR_DELAY);
	
	/* PLANK DETECTION ALGORITHM */
	
	// // find maximum reading over 0.75 plank revolutions
	// plank_max = 0;
	// for(i = 0; i < IR_NUM_SAMPLES; i++) {
		// plank = IR_read_dist();
		// if(plank > plank_max) { plank_max = plank; }
		// sleep(IR_SAMP_INTERVAL);
	// }
	// fprintf_P(&debug, PSTR("Peak IR = %u\r\n"), plank_max);
	
	// calc. thresholds
	plank_THR_UP = 400;
	plank_THR_DN = 280;
	
	// wait for negative transition
	//PT_WAIT_UNTIL(pt, IR_read_dist() > plank_THR_UP);
	PT_WAIT_UNTIL(pt, IR_read_dist() < plank_THR_DN);
	
	// get center time and pass a fixed time after
	PT_WAIT_UNTIL(pt, IR_read_dist() > plank_THR_UP); t_UP = uptime();
	PT_WAIT_UNTIL(pt, IR_read_dist() < plank_THR_DN); t_DN = uptime();
	t_pass = t_DN - ((t_DN - t_UP) >> 1) + CLEARANCE_DELAY;
	if(t_pass < t_DN) { t_pass = t_DN; }
	PT_WAIT_WHILE(pt, uptime() < t_pass);
	
	/* END PLANK DETECTION ALGORITHM */
	
	// old algorithm... just in case
	/*
	PT_WAIT_UNTIL(pt, IR_read_dist() < PLANK_THRESHOLD_DN);
	PT_WAIT_UNTIL(pt, IR_read_dist() > PLANK_THRESHOLD_UP); t_UP = uptime();
	PT_WAIT_UNTIL(pt, IR_read_dist() < PLANK_THRESHOLD_DN); t_DN = uptime();
	t_diff = (t_DN - t_UP) >> 1;
	PT_WAIT_UNTIL(pt, IR_read_dist() > PLANK_THRESHOLD_UP);
	PT_WAIT_UNTIL(pt, IR_read_dist() < PLANK_THRESHOLD_DN); t_DN = uptime();
	t_diff = t_DN - t_diff + CLEARANCE_DELAY;
	PT_WAIT_WHILE(pt, uptime() < t_diff);
	*/
	
	forward(50); sleep(500);
	forward(120); servo1(75);
	sleep(2000); // required to prevent accelerometer detect speed-up
	PT_WAIT_UNTIL(pt, is_climbing(30.0));
	fprintf(&debug, "\a");
	
	//FIXME: this value must be calibrated
	//delay after the root passes the 30 degree treshold before it deploys its rear servo
	sleep(200); //original value: 1000
	servo1(120);
	sleep(1500); 
	stop(); sleep(1000);
	
	// got up the platform here
	servo1(SERVO_MIN); sleep(500);
	
	forward(120); sleep(3000); //FIXME: find right value so the ball is pushed
	stop(); sleep(100);
	//stop EVERYTHING. END OF EXECUTION
	/*
	freedrive(20, 0, 0x10); sleep(800);
	
	forward_dist(50, 250); PT_WAIT_UNTIL(pt, drive_complete); // grab the ball
	stop(); sleep(100);
	disable_rotadj();
	orient_self(180); PT_WAIT_UNTIL(pt, drive_complete);
	stop(); sleep(100);
	enable_rotadj();
	
	// get off the platform
	forward_dist(70, 250); PT_WAIT_UNTIL(pt, drive_complete);
	stop(); sleep(100);
	backwards_dist(50, 30); PT_WAIT_UNTIL(pt, drive_complete);
	stop(); sleep(100);
	orient_self(160); PT_WAIT_UNTIL(pt, drive_complete);
	
	// get to ledge
	IR_enable_F(); sleep(SENSOR_DELAY);
	forward(25); PT_WAIT_UNTIL(pt, IR_hole_F());
	stop(); sleep(100);
	IR_disable_F(); sleep(SENSOR_DELAY);
	
	// drop-off the mutherfucking ball
	IR_enable_R(); sleep(SENSOR_DELAY);
	freedrive(0, 50, 0x00); PT_WAIT_UNTIL(pt, IR_hole_R());
	stop(); sleep(100);
	IR_disable_R(); sleep(SENSOR_DELAY);
	
	// bitches that's it!
	sleep(8000);
	*/
	PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required as a denote the end of a thread
}

float read_acc_mag2(void) {
	float acc_v[3];
	read_acc_values(acc_v);
	return acc_v[0]*acc_v[0] + acc_v[1]*acc_v[1] + acc_v[2]*acc_v[2];
}

uint8_t is_falling(float fall_threshold)
{
    // we're only looking at the Z-angle to determine fall
    float acc_a[3];
    read_acc_angles(acc_a); // read angles only
    return (acc_a[2] < fall_threshold) ? 1 : 0;
}

uint8_t is_climbing(float climb_threshold)
{
    // we're only looking at the X-angle to determine climb
    float acc_a[3];
    read_acc_angles(acc_a); // read angles only
    return (acc_a[0] > climb_threshold) ? 1 : 0;
}

void stop()                                        { fprintf_P(&drive, PSTR("30\r"));              }
void forward(uint8_t speed)                        { fprintf_P(&drive, PSTR("31%02x00\r"), speed); }
void backwards(uint8_t speed)                      { fprintf_P(&drive, PSTR("31%02x11\r"), speed); }
void turn_left(uint8_t speed)                      { fprintf_P(&drive, PSTR("31%02x10\r"), speed); }
void turn_right(uint8_t speed)                     { fprintf_P(&drive, PSTR("31%02x01\r"), speed); }
void freedrive(uint8_t speedL, uint8_t speedR, uint8_t dir) { fprintf_P(&drive, PSTR("34%02x%02x%02x\r"), speedL, speedR, dir); }
void disable_rotadj()                              { fprintf_P(&drive, PSTR("4300\r")); }
void enable_rotadj()                               { fprintf_P(&drive, PSTR("4301\r")); }
// commands below return acknowledgement: "D\r"
void forward_dist(uint8_t speed, uint16_t dist)    { drive_complete = 0; fprintf_P(&drive, PSTR("32%02x%04x\r"), speed, dist); }
void backwards_dist(uint8_t speed, uint16_t dist)  { drive_complete = 0; fprintf_P(&drive, PSTR("33%02x%04x\r"), speed, dist); }
void turn_ccw_by(uint16_t angle)		   { drive_complete = 0; fprintf_P(&drive, PSTR("3520%04x\r"),   angle);       }
void turn_cw_by(uint16_t angle)                    { drive_complete = 0; fprintf_P(&drive, PSTR("3620%04x\r"),   angle);       }
void orient_self(int16_t angle)                    { drive_complete = 0; fprintf_P(&drive, PSTR("3720%04x\r"),   angle);       }

float get_heading()
{
    float acc_v[3];
    read_acc_values(acc_v);
    return atan2(acc_v[0], acc_v[1]) * 180.0 / M_PI;
}

void orient_self_0()
{
    double curr_heading = get_heading();
    if (curr_heading > 180)
	turn_ccw_by( (int)(curr_heading)); //10th of an angle...
    else
	turn_cw_by( (int)(curr_heading ) ); //10th of an angle...
}
void orient_self_90()
{
    double curr_heading = get_heading();
    if (curr_heading > 90 && curr_heading<270)
	turn_cw_by( (int)(curr_heading - 89.5)); //10th of an angle...
    else if (curr_heading <= 90)
	turn_ccw_by( (int)(90 - curr_heading ) ); //10th of an angle...
    else
	turn_ccw_by((int) ( 90 + (360-curr_heading) ));
}
void orient_self_180()
{
    double curr_heading = get_heading();
    if (curr_heading > 180.0)
	turn_cw_by( (int)(curr_heading - 179.5)); //10th of an angle...
    else
	turn_ccw_by( (int)(179.5 - curr_heading ) ); //10th of an angle...
}
void orient_self_270()
{
    double curr_heading = get_heading();
    if (curr_heading > 90 && curr_heading<270)
	turn_ccw_by( (int)(270 - curr_heading)); //10th of an angle...
    else if (curr_heading <= 90)
	turn_cw_by( (int)(90 + curr_heading ) ); //10th of an angle...
    else
	turn_cw_by((int) (curr_heading - 270) );
}

void check_drive_uart(void) {
	static uint8_t inputbuf[RX_LINE_SIZE], inputptr = 0;
	uint8_t i, recv;
	
	while(uart_available(DRIVE)) {
		recv = uart_get(DRIVE);
		
		if(recv == '\r') {
			if(inputptr) {
				switch(inputbuf[0]) {
					case 'D':
					if(inputptr == 1) { drive_complete = 1; }
					break;
					
					case 'T':
					// TODO: get distance travelled
					break;
					
					default:
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
	float acc[3];
	uint8_t i, j, recv;
	
	while(uart_available(DEBUG)) {
		recv = uart_get(DEBUG);
		
		if(recv == '\r') {
			fprintf(&debug, "\r\n");
			
			if(inputptr) {
				switch(inputbuf[0]) {
					case '?': // print drive command list
					fprintf_P(&debug, PSTR("stop() ......................... | p30\r\n"));
					fprintf_P(&debug, PSTR("forward(speed) ................. | p31%02x00\r\n"));
					fprintf_P(&debug, PSTR("backwards(speed) ............... | p31%02x11\r\n"));
					fprintf_P(&debug, PSTR("turn_left(speed) ............... | p31%02x10\r\n"));
					fprintf_P(&debug, PSTR("turn_right(speed) .............. | p31%02x01\r\n"));
					fprintf_P(&debug, PSTR("forward_dist(speed, dist) ...... | p32%02x%04x\r\n"));
					fprintf_P(&debug, PSTR("backwards_dist(speed, dist) .... | p33%02x%04x\r\n"));
					fprintf_P(&debug, PSTR("freedrive(speedL, speedR, dir) . | p34%02x%02x%02x\r\n"));
					fprintf_P(&debug, PSTR("orient_self(angle) ............. | p3720%04x\r\n"));
					fprintf_P(&debug, PSTR("disable_rotadj() ............... | p4300\r\n"));
					fprintf_P(&debug, PSTR("enable_rotadj() ................ | p4301\r\n"));
					break;
					
					case 'p': // passthrough to DRIVE MCU
					for(i = 1; i < inputptr; i++) { uart_put(DRIVE, inputbuf[i]); }
					uart_put(DRIVE, '\r');
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
					
					case 'l': // left sensor commands
					if(inputptr == 1) { fprintf(&debug, "%c\r\n", IR_hole_L() | '0'); }
					else if(inputbuf[1] == '0') { IR_disable_L(); }
					else if(inputbuf[1] == '1') { IR_enable_L(); }
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case 'r': // right sensor commands
					if(inputptr == 1) { fprintf(&debug, "%c\r\n", IR_hole_R() | '0'); }
					else if(inputbuf[1] == '0') { IR_disable_R(); }
					else if(inputbuf[1] == '1') { IR_enable_R(); }
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case 'f': // front sensor commands
					if(inputptr == 1) { fprintf(&debug, "%c\r\n", IR_hole_F() | '0'); }
					else if(inputbuf[1] == '0') { IR_disable_F(); }
					else if(inputbuf[1] == '1') { IR_enable_F(); }
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case 'd': // distance sensor commands
					if(inputptr == 1) { fprintf(&debug, "%u\r\n", IR_read_dist()); }
					else if(inputbuf[1] == '0') { IR_disable_dist(); }
					else if(inputbuf[1] == '1') { IR_enable_dist(); }
					else if(inputbuf[1] == '2') { print_distance ^= 1; }
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case 'a': // accelerometer commands
					if(inputbuf[1] == 'a') {
						read_acc_angles(acc);
						fprintf_P(&debug, PSTR("%+f\t%+f\t%+f\r\n"), acc[0], acc[1], acc[2]);
					}
					else if(inputbuf[1] == 'v') {
						read_acc_values(acc);
						fprintf_P(&debug, PSTR("%+f\t%+f\t%+f\r\n"), acc[0], acc[1], acc[2]);
					}
					else if(inputbuf[1] == 'h') { fprintf_P(&debug, PSTR("%+f\r\n"), get_heading()); }
					else if(inputbuf[1] == '0') { print_acc = 0; }
					else if(inputbuf[1] == '1') { print_acc = 1; }
					else if(inputbuf[1] == '2') { print_acc = 2; }
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case 'e': // electromagnet commands
					if(inputptr == 1) {
						if(EM_status()) { EM_disable(); fprintf_P(&debug, PSTR("EM disabled!\r\n")); }
						else            { EM_enable();  fprintf_P(&debug, PSTR("EM enabled!\r\n"));  }
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '1': // servo1 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo1(htoa(inputbuf[1], inputbuf[2]));
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '2': // servo2 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo2(htoa(inputbuf[1], inputbuf[2]));
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case 'w': // write test settings
					if(inputptr > 2 && inputptr <= 22 && !(inputptr & 1)) {
						for(i = 0; i < 10; i++) {
							settings[htoa(0, inputbuf[1])][i] = htoa((i << 1) + 2, (i << 1) + 3);
						}
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case 'm': // show test settings
					for(i = 0; i < 10; i++) {
						for(j = 0; j < 10; j++) { fprintf(&debug, "%02x ", settings[i][j]); }
						fprintf(&debug, "\r\n");
					}
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

void check_acc_update(void) {
	set_interval(20);
	acc_update();
	fprintf_P(&drive, PSTR("41%04x\r"), (int16_t)get_heading());
}

void check_print_stat(void) {
	set_interval(20);
	float acc[3];
	if(print_distance) { fprintf_P(&debug, PSTR("%u\r\n"), IR_read_dist()); }
	if(print_acc == 1) {
		read_acc_values(acc);
		fprintf_P(&debug, PSTR("%+f\t%+f\t%+f\r\n"), acc[0], acc[1], acc[2]);
	}
	if(print_acc == 2) {
		read_acc_angles(acc);
		fprintf_P(&debug, PSTR("%+f\t%+f\t%+f\r\n"), acc[0], acc[1], acc[2]);
	}
}

