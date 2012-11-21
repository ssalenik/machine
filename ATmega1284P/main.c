#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
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
void IR_enable_L(void);
void IR_enable_R(void);
void IR_enable_F(void);
void IR_enable_dist(void);
void IR_disable_L(void);
void IR_disable_R(void);
void IR_disable_F(void);
void IR_disable_dist(void);

// "public" functions of "uart.c"
void init_uart(void);
// NOTE: to print via uart, use: fprintf_P(&{drive|debug}, PSTR("format_string"), vars...);

// "public" functions of "acc.c"
void init_acc(void);
void read_acc_values(float *values);
void read_acc_angles(float *angles);

// "public" functions of "adc.c"
void init_adc(void);
uint16_t IR_read_dist(void);

// "public" functions of "timer.c"
void init_timer(void);
uint32_t uptime(void);
void sleep(uint16_t seconds);

/*! END FUNCTION PROTOTYPES OF INCLUDED FILES */


#include "delay.c"
#include "hex.c"
#include "ports.c"
#include "uart.c"
#include "acc.c"
#include "adc.c"
#include "timer.c"


#define fast 10
#define slow 5
#define stop 0
#define IR_DIST 100 //hardcoded value... distance from the front IR when we must stop.
#define fall_treshhold 30 //in degrees

float before_fall[3];

void init();
float InvSqrt(float x);
//int get_speed_left();
//int get_speed_right();
int is_falling();
void fall_ledge();
double get_heading();
void orient_self_0();
void orient_self_90();
void orient_self_180();
void orient_self_270();
void navigate_lab();
int COMPUTATION = 0;

int main()
{
    init();
    fall_ledge();
//    orient_self_180();
//    navigate_lab();
//    bridge();
    
    return 0;
}

void init(void) {
	init_ports();
	
	delay_ms(T_MS * 200); // wait for capacitors to charge
	
	init_uart();
	init_acc();
	init_adc();
	init_timer();
	
	sei(); // enable interrupts
}

float InvSqrt(float x) {
	union { float f32; long s32; } i;
	float xhalf = 0.5f * x;
	// int i = *(int*)&x;
	i.f32 = x;
	i.s32 = 0x5f3759d5 - (i.s32 >> 1); // initial guess for Newton's method
	//x = *(float*)&i;
	x = i.f32 * (1.5f - xhalf * i.f32 * i.f32); // One round of Newton's method
	return x;
}

/*
int get_speed_left()
{
    return 0;
}

int get_speed_right()
{
    return 0;
}*/

int is_falling()
{
    //we're only looking at the Z-angle to determine fall
    float during_fall[3];
    read_acc(NULL, during_fall); // read angles only
    if ( (before_fall[2] - during_fall[2]) > fall_treshhold )
	return 1;
    else 
	return 0;
    
}
void fall_ledge()
{
    go_backwards(slow); //millimeters
    COMPUTATION = 1;
    sleep(3);
    stop();
    //sleep(3); //5 seconds should be enough to cover that dist. given the immediate return...
    //stop(); //just in case.
    //set values before the fall to be compared with to determine start of fall.
    read_acc(NULL, before_fall); // read angles only
    go_forward(slow);
    //now the robot moves forward. we need to cut the motors when it starts falling down, so it doesn't roll off the floor after landing
    while(!is_falling())
    {
	;
    }
    //it is falling. we stop the motors, and wait for it to land
    stop();
//    COMPUTATION = 0;
    sleep(3);

}

double get_heading()
{
    double val=atan2(accelerometer_angles[0], accelerometer_angles[1]);
    return val*180.0/(M_PI);
}

void orient_self_0()
{
    double curr_heading = get_heading();
    if (curr_heading > 180)
	turn_left_angle( int(curr_heading)); //10th of an angle...
    else
	turn_right_angle( int(curr_heading ) ); //10th of an angle...
    sleep(10);
    stop();
}
void orient_self_90()
{
    double curr_heading = get_heading();
    if (curr_heading > 90 && curr_heading<270)
	turn_right_angle( int(curr_heading - 89.5)); //10th of an angle...
    else if (curr_heading <= 90)
	turn_left_angle( int(90 - curr_heading ) ); //10th of an angle...
    else
	turn_left_angle(int ( 90 + (360-curr_heading) );
    sleep(10);
    stop();
}
void orient_self_180()
{
    double curr_heading = get_heading();
    if (curr_heading > 180.0)
	turn_right_angle( int(curr_heading - 179.5)); //10th of an angle...
    else
	turn_left_angle( int(179.5 - curr_heading ) ); //10th of an angle...
    sleep(10);
    stop();
}
void orient_self_270()
{
    double curr_heading = get_heading();
    if (curr_heading > 90 && curr_heading<270)
	turn_left_angle( int(270 - curr_heading)); //10th of an angle...
    else if (curr_heading <= 90)
	turn_right_angle( int(90 + curr_heading ) ); //10th of an angle...
    else
	turn_right_angle(int (curr_heading - 270) );
    sleep(10);
    stop();
}


void navigate_lab() {

    //we start oriented in the 180 degree direction. 
    //step 1: move until we reach the wall
    
    COMPUTATION = 1;
    go_forward(fast);
    while ( IR_read_dist() > IR_DIST)
    {
	//advance 
	if ( IR_hole_R() )
	{
	    //if there's a hole on the right (close), we move to the left
	    stop();
	    turn_left_angle(15);
	    sleep(3); //wait for operation to complete
	    go_forward(fast);
	    sleep(1);
	}
    }
    stop();
    
    //step 2: rotate to look north.
    orient_self_90();
    go_forward(fast);
    
    while ( !IR_hole_F() )
    {
	if ( IR_hole_R() )
	{
	    //if there's a hole on the right (close), we move to the left
	    stop();
	    turn_left_angle(15);
	    sleep(3); //wait for operation to complete
	    go_forward(fast);
	    sleep(1);
	}
    }

    //reached target. now move right... until when?
    orient_self_180();
    //move backwards, while looking at all 3 sensors - 2 sides to avoid  falling (NOTE: REVERSED) and the front so that we know how far we are

    COMPUTATION = 0;
}

