
/* Includes */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "uart.h"
#include "hex.h"


/* -------- */




/*  Defines  */

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define MAXTIMER 4000 // 1600 at 8Mhz, 4000 at 20Mhz
// This is the max value to be placed in the OC1x register
#define POWERCAP 64 // caps the maximum power in %
#define MAXPOWER (MAXTIMER / 100 * POWERCAP)
#define MAXPOW100 POWERCAP
#define MAXPOW800 (POWERCAP * 8)

#define RATEPID 5 // Period of PID calculations in ms

// Thresholds for ADC reads from sensors.
// Hysterysis implemented to have noise-resistant
// low-high and high-low transitions
// Thresholds can be different for channel 0 and 1
#define A_LOW0 100
#define A_HIGH0 160
#define A_LOW1 100
#define A_HIGH1 160
// Consecutive reads of same value are required for
// state change. Implemented for further noise reduction
#define A_COUNT 3

/* define motor pins here
 for LMOTOR and RMOTOR, values are 
  - 0 for OC1A
  - 1 for OC1B
 for M0IN1, M0IN2, M1IN1, M1IN2, pins are:
  - All on port b
	- pins 0, 3, 4, 5
	- M0IN1 & M01N2 are direction pins for motor 0 / OC1A
	- M1IN1 & M11N2 are direction pins for motor 1 / OC1B
*/

#define LMOTOR 1
#define RMOTOR 0

#define LADC 1			// LADC is the ADC channel related to left motor
#define RADC 0			// RADC is the ADC channel related to right motor

#define M0IN1 3
#define M0IN2 0
#define M1IN1 5
#define M1IN2 4

#define FORWARD 0   // High-Low
#define BACKWARD 1  // Low-High 
#define BRAKE 2     // High-High
#define NEUTRAL 3   // Low-Low

#define ENDCHAR '\r'

// conversions between ticks and distance in mm (value is >> 8)
// basically, 1 ticks = 0.67 mm
#define DISTTOTICKS 381
#define TICKSTODIST 172
// basically, 1 ticks = 0.64 degrees
#define ANGLETOTICKS 400
// Slip adjustment
#define SLIPADJ0 136
#define SLIPADJ90 280
#define SLIPADJ180 120
#define SLIPADJ270 248

#define ANG_CW 0
#define ANG_CCW 1
/* -------- */



/* Function Prototypes */

void initADC();
uint8_t readADC(uint8_t channel);
void initPWM0();
void initPWM1();
void setPower(uint8_t motor, uint16_t power);
void setPowerA(uint8_t motor, uint8_t power100);
void setPowerB(uint8_t motor, uint16_t power800);
void setDirection(uint8_t motor, uint8_t direction);
void setTargetSpeed(uint8_t motor, int16_t speed);
void initMotorPins();
void readCommand();
inline uint8_t readByte(char *buf, uint8_t *valid);
inline int16_t readInt(char *buf, uint8_t *valid);
inline uint16_t readUInt(char *buf, uint8_t *valid);
void runPID();
void resetPID();
void printParams();
void printParams2();
void navigator();
void navSync(int16_t speed, uint8_t dirL, uint8_t dirR);
void navDist(int16_t speed, uint8_t dir, int16_t distance);
void navFree(int16_t speedL, int16_t speedR, uint8_t dirL, uint8_t dirR);
void navRot1(int16_t speed, int16_t angle, uint8_t dir);
void navRot2(int16_t speed, int16_t tHeading);
int16_t deltaAng(int16_t startAng, int16_t endAng, uint8_t direction);
int16_t adjustAng(int16_t angle);
void sendDone();
void sendDist();
void slipAdjust();
uint8_t angleWithin(int16_t angle1, int16_t angle2, int16_t maxDelta);


/* -------- */



/* Variables */

volatile uint8_t interrupt_prescaler = 0;
volatile uint8_t chan = 0; //ADC channel to read during iterrupt
volatile uint8_t ch0, ch1; //ADC reads from interrupt

/* state is the current digital state of the ADC reading
   if ADC reading falls outside of the analog thresholds
   A_LOW or A_HIGH of that state, count is increased.
   Once count reaches A_count, state is changed.
   For every Low-high transition, ticks is increased. */
volatile uint8_t state0 = 0, state1 = 0;
volatile uint8_t count0 = 0, count1 = 0;
volatile int32_t ticks0 = 0, ticks1 = 0;

volatile int32_t timer8 = 0; // timer in 1/8th of a millisecond
volatile int32_t timer = 0; // timer in milliseconds

uint8_t ldir = FORWARD;
uint8_t rdir = FORWARD;

// speed calculations
volatile int32_t tickTimes0[4], tickTimes1[4]; // timestamps of previous ticks in 1/8th of ms
volatile uint8_t tickInd0 = 0, tickInd1 = 0;
volatile uint16_t period0 = 0, period1 = 0; // period between 4 ticks

/* This statement allows printf to work with serial com
   for every character sent to stream uart_stdout, uart_put is executed
   later in code, stdout is set to uart_stdout, so that
   printf writes to the stream uart_stdout automatically */
static FILE uart_stdout = FDEV_SETUP_STREAM(uart_put, NULL, _FDEV_SETUP_WRITE);

// PID parameters
uint8_t kP = 32; // P Constant
uint8_t kI = 16;  // I Constant
uint8_t kD = 64;  // D Constant
uint8_t kX = 64;  // Cross dependency between both motor displacements
int16_t errIMax = 80;    // Max Integer value
int16_t errIMin = -80;
int16_t adjustMax = 30;  // Max power adjustment factor
int16_t adjustMin = -20;
int16_t adjustXMax = 50; // Max power adjustment from kX
int16_t adjustXMin = -50; 
uint8_t xCalibration = 128; // calibrates motor1 to motor0 using ticks. 128 for no calibration

// PID variables
uint8_t adjXOn = 0; // enable or disable cross adjustment (manual)
//uint8_t useAdjX = 0; // enable or disable cross adjustment (auto)
uint8_t pidOn = 1;   // turns PID on or off
int32_t lastPID = 0; // PID timer
// Note: Speed is in ticks / sec, where 1 tick = 0.6 mm
//       Speed is only calculated when PID is on
uint16_t speed0 = 0, speed1 = 0; // speed of motor in ticks / sec
uint16_t targetSpeed0 = 0, targetSpeed1 = 0; // target speed ofrobot
uint16_t lastSpeed0[8], lastSpeed1[8]; // values for the D part
uint16_t lastSpeedInd0 = 0, lastSpeedInd1 = 0; // index for the current value of lastSpeed
int16_t errI0 = 0, errI1 = 0; // value for the I part
uint16_t power0 = 0, power1 = 0; // power (0 to 100 * 256) applied to motor
int16_t adjustX = 0;

// Navigation variables
typedef enum {
	NAV_NONE = 0,
	NAV_SYNC = 1,
	NAV_DIST = 2,
	NAV_FREE = 3,
	NAV_ROT  = 4,
	NAV_DCHK = 5,
	NAV_RCHK = 6
} NavCom;
NavCom navCom = NAV_NONE;
int16_t heading = 0; // current heading of the robot as recieved from master
int16_t n_ticks = 0; // target distance in ticks to travel by navigator when in mode NAV_DIST
int16_t n_slowTicks = 100; // distance in ticks before slowing down to n_slowSpeed if targetSpeed > n_slowSpeed
int16_t n_stopTicks = 8; // distance in ticks when to stop if targetSpeed = n_slowSpeed
int16_t n_slowSpeed = 50;
int16_t n_targetHeading = 0; // target heading to travel by navigator when in mode NAV_ROT
//int16_t n_angleTarget = 0; // target angle to travel by navigator when in mode NAV_ROT
//int16_t n_angle = 0; // angle travelled in mode NAV_ROT
//int16_t n_lastHeading = 0; // last heading reading by navigator
//int16_t n_slowAngle = 10; // angle from target heading before slowing down if targetSpeed > n_slowASpeed;
int16_t n_stopAngle = 2; // angle from target heading when to stop if targetSpeed = n_slowASpeed
int16_t n_slowATicks = 50;
int16_t n_stopATicks = 8;
int16_t n_slowASpeed = 35;
int16_t n_wait = 500; // wait in ms before checking that heading is same as target
int32_t n_timer = 0;
uint8_t n_rot_dir = ANG_CW;  // direction of rotation for NAV_ROT
uint8_t slipAdjFOn = 0;
uint8_t slipAdjSOn = 0;
uint8_t rotAdjOn = 1;

// Debug
uint8_t debug1 = 0;
uint8_t debug2 = 0;
uint8_t debug3 = 0;
uint8_t debug4 = 0;
uint8_t debug5 = 0;
uint8_t debug6 = 0;
uint8_t debug7 = 0;
uint16_t d_count1 = 0;
void debugF1();

/* -------- */


int main(void) {
	//uint32_t power;
	int32_t lastTime = 0;
	uint8_t i = 0;

	//OSCCAL = 0x58; // internal oscillator frequency calibration
	//debugF1();
	//slipAdjOn = 1;
	//heading = -176;
	//navDist(50, 0, 1000);
	//heading = -2;
	//navDist(50, 0, 1000);
	//heading = 88;
	//navDist(50, 0, 1000);
	//heading = -93;
	//navDist(50, 0, 1000);

	stdout = &uart_stdout;

	for (i=0; i<4;i++) {
		tickTimes0[i] = 0;
		tickTimes1[i] = 0;
	}

	resetPID();

	uart_init();

	initPWM1();
	initADC();
	initMotorPins();
	sbi(DDRC, 3);
	sbi(DDRC, 4);

	setDirection(LMOTOR, FORWARD);
	setDirection(RMOTOR, FORWARD);
	setPowerA(LMOTOR, 0);
	setPowerA(RMOTOR, 0);

	sei(); // enable interrupts

	while(1) {
		readCommand();
		if (pidOn) {
			if (timer - lastPID >= RATEPID) {
				lastPID = timer;
				navigator();
				runPID();
			}
			
		}
		if (timer - lastTime >= 100L) {
			lastTime += 100;
			//printf("%lu\t%lu\r\n", ticks0, ticks1);
			if (debug7) printf("%u\t%u\r\n", power0, power1);
			if (debug6) printf("%u\t%u\r\n", speed0, speed1);
			if (debug5) printf ("%lu\t%lu\r\n", ticks0, ticks1);
			if (debug3) printf ("%d\t%d\r\n", heading, n_targetHeading);
			//printf("%u\t%u\r\n", period0, period1);
		}
	}
	
	while(1);
}

void debugF1() {
	debug1 = angleWithin(179, -165, 20);
	debug2 = angleWithin(-3, 179, 20);
	debug3 = angleWithin(13, -175, 20);
	debug4 = angleWithin(-176, 178, 20);
	debug5 = 0;
}

// sets the power of the motor. To use with timer1.
// - 0 for pin PB1 / OC1A
// - 1 for pin PB2 / OC1B
// ranges from 0 to MAXPOWER
void setPower(uint8_t motor, uint16_t power) {
	if (power > MAXPOWER) power = MAXPOWER;
	if (motor) {
		OCR1B = MAXTIMER - power; // opposite phase
	} else {
		OCR1A = power;
	}	
}


// Same as setPower, but ranges from 0-100 for ease of use
// ranges from 0 to 100
void setPowerA(uint8_t motor, uint8_t power100) {
	if (power100 > MAXPOW100) power100 = MAXPOW100;
	if (motor) {
		OCR1B = MAXTIMER - (uint16_t)power100 * (MAXTIMER/100); // opposite phase
	} else {
		OCR1A = (uint16_t)power100 * (MAXTIMER/100);
	}	
}


// Same as setPower, but ranges from 0-800 for PID controller
// ranges from 0 to 800
void setPowerB(uint8_t motor, uint16_t power800) {
	if (power800 > MAXPOW800) power800 = MAXPOW800;
	if (motor) {
		OCR1B = MAXTIMER - power800 * (MAXTIMER/800); // opposite phase
	} else {
		OCR1A = power800 * (MAXTIMER/800);
	}	
}


// define direction of a motor
// direction can be: FORWARD, BACKWARD, BRAKE or NEUTRAL
void setDirection(uint8_t motor, uint8_t direction) {
	if (motor == LMOTOR) {
		ldir = direction;
	} else {
		rdir = direction;
	}

	if (motor) {
		switch(direction) {
		case FORWARD:
			sbi(PORTB, M1IN1);
			cbi(PORTB, M1IN2);
			break;
		case BACKWARD:
			cbi(PORTB, M1IN1);
			sbi(PORTB, M1IN2);
			break;
		case BRAKE:
			sbi(PORTB, M1IN1);
			sbi(PORTB, M1IN2);
			break;
		default:
			cbi(PORTB, M1IN1);
			cbi(PORTB, M1IN2);
		}
	} else {
		switch(direction) {
		case FORWARD:
			sbi(PORTB, M0IN1);
			cbi(PORTB, M0IN2);
			break;
		case BACKWARD:
			cbi(PORTB, M0IN1);
			sbi(PORTB, M0IN2);
			break;
		case BRAKE:
			sbi(PORTB, M0IN1);
			sbi(PORTB, M0IN2);
			break;
		default:
			cbi(PORTB, M0IN1);
			cbi(PORTB, M0IN2);
		}
	}
}

void setTargetSpeed(uint8_t motor, int16_t speed) {
	if (motor) {
		targetSpeed1 = speed;
	} else {
		targetSpeed0 = speed;
	}
}

// Inits PWM on timer 0
// NOTE: NOT USED
void initPWM0() {
	TCCR0A = 0b10100001; 	// enable PWM0A, PWM0B
	TCCR0B = 0b00000010; 	// 1.95kHz at 8Mhz clock
	OCR0A = 64;
	OCR0B = 192;
	//DDRD =   0b01100000; 	// set PWM0A, PWM0B pins as output pins
}

// Inits PWM on timer 1
void initPWM1() {
	TCCR1B |= 0b00010000; // set mode to Phase & freq Correct PWM
	TCNT1 = MAXTIMER >> 1; // set timer to middle to prevent output pulse
	ICR1 = MAXTIMER; // fmotor = F_CPU / 2 / MAXTIMER
	OCR1A = 0; // set output to 0 on both channels
	OCR1B = MAXTIMER;
	TCCR1B |= 0b00000001; // set prescaler to 1 (start TIMER1)
	TCCR1A =  0b10110000; // enable PWM1A, PWM1B in opposite phase
	DDRB |= 0b00000110; // set PWM pins A and B as output
}

// Inits params for ADC functioning, including timer0
void initADC() {

	// configure timer0 interrupt for polling
	TCCR0A = 0b00000010;	// set timer to CTC mode	
	TCCR0B = 0b00000001;	// set clock prescaler to 8
	OCR0A = 249;			// set to 124 + 1 timer ticks per interrupt
	// total frequency is 8kHz at 8Mhz clock (8M / 8 / 125 = 8k)

	TIMSK0 |= 1 << OCIE0A;// enable compare A interupt

	// configure ADC module
	ADMUX |= 1 << REFS0; 	// set ref. voltage to AVcc
	ADMUX |= 1 << ADLAR;  // left justify output	
	
	// Note: ADC channel is set in bits 3..0 of ADMUX. 0000 for chan0

	ADCSRA |= 7;		  // set ADC clock prescaler to 64
	ADCSRA |= 1 << ADEN;  // enable ADC
}

void initMotorPins() {
	DDRB |=  0b00111001;	// set pins 0, 3, 4, 5 as output in PORT B
	PORTB &= 0b11000110;	// set the values of those pins to 0
}

uint8_t readADC(uint8_t channel) {
	channel &= 0x0f;
	ADMUX &= 0xf0;
	ADMUX |= channel; 		// choose channel
	ADCSRA |= 1 << ADSC;	// start conversion	
	while (ADCSRA & (1 << ADSC)); // wait for convertion to finish
	return ADCH;
}

// Timer0 COMPARE A interrupt used for the ADC
ISR(TIMER0_COMPA_vect) {
	// the interrupt assumes the ADC read was started before and has completed
	// the result of the read is stored in the register ADCH
	// channel 0 and 1 are read alternatingly
	int32_t tmp;
	char deb[2]; // *** debug ***

	interrupt_prescaler++;
	if (interrupt_prescaler == 10) {
		interrupt_prescaler = 0;
	} else {
		return;
	}

	if (chan) {
		ch1 = ADCH; 			// fill ch1 with value from previous ADC read
		ADMUX &= 0xf0;  	// select channel 0 for next read
		chan = 0;
		
		/*if (debug3) {
			if (++d_count1 >= 2500) {
				d_count1 = 0;
				debug3 = 0;
			}
			//printf("%d\r\n", ch1);
			if (d_count1 & 1) {
				atoh(ch1, deb);
				uart_put(deb[0], NULL);
				uart_put(deb[1], NULL);
				uart_put('\n', NULL);
				uart_put('\r', NULL);
			}
		} */

		if (state1) {
			// check for high to low tranistion
			if (ch1 < A_LOW1) {
				count1++;
				if (count1 >= A_COUNT) {
					state1 = 0;
					count1 = 0;
				}
			} else {
				count1 = 0;
			}
		} else {
			// check for low to high tranistion
			if (ch1 > A_HIGH1) {
				count1++;
				if (count1 >= A_COUNT) {
					state1 = 1;
					count1 = 0;
					ticks1++; // increase ticks on every low-high transition
					// calculate speed (or period to be more precise)
					tmp = timer8 - tickTimes1[tickInd1];
					tickTimes1[tickInd1] = timer8;
					if (tmp < 1600) {
						period1 = tmp;
						if (debug1) printf("%d\r\n", period1);
					} else {
						period1 = 0;
					}
					tickInd1++;
					tickInd1 &= 0x03; // circular buffer
				}
			} else {
				count1 = 0;
			}
		}
	} else {
		ch0 = ADCH; 			// fill ch0 with value from previous ADC read
		ADMUX &= 0xf0;  
		ADMUX |= 0x01;		// select channel 1 for next read
		chan = 1;

		if (debug4) {
			if (++d_count1 >= 2500) {
				d_count1 = 0;
				debug4 = 0;
			}
			if (d_count1 & 1) {
				atoh(ch0, deb);
				uart_put(deb[0], NULL);
				uart_put(deb[1], NULL);
				uart_put('\n', NULL);
				uart_put('\r', NULL);
			}
		} 

		if (state0) {
			// check for high to low tranistion
			if (ch0 < A_LOW0) {
				count0++;
				if (count0 >= A_COUNT) {
					state0 = 0;
					cbi(PORTC, 3);
					count0 = 0;
				}
			} else {
				count0 = 0;
			}
		} else {
			// check for low to high tranistion
			if (ch0 > A_HIGH0) {
				count0++;
				if (count0 >= A_COUNT) {
					state0 = 1;
					sbi(PORTC, 3);
					count0 = 0;
					ticks0++; // increase ticks on every low-high transition
					// calculate speed (or period to be more precise)
					tmp = timer8 - tickTimes0[tickInd0];
					tickTimes0[tickInd0] = timer8;
					if (tmp < 1600) {
						period0 = tmp;
						if (debug2) printf("%d\r\n", period0);
					} else {
						period0 = 0;
					}
					tickInd0++;
					tickInd0 &= 0x03; // circular buffer
				}
			} else {
				count0 = 0;
			}
		}
	}

	// every 128 ms, check if wheels are still truning to adjust period
	if ((timer & 0x7f) == 0) {
		tmp = timer8 - tickTimes0[tickInd0];
		if (tmp > 1600) {
			period0 = 0;
		}
		tmp = timer8 - tickTimes1[tickInd1];
		if (tmp > 1600) {
			period1 = 0;
		}
	}
	
	sbi(ADCSRA, ADSC);	// start ADC read
	timer8++;
	timer = timer8 >> 3;
}

void readCommand() {
	static char buf[32];
	static uint8_t index = 0;
	//static char tst[] = "0128\rxxx"; // ***debug***
	//uint8_t tsti = 0; // ***debug***
	char c, cmd;
	uint8_t arg1, arg2, arg3, arg4;
	int16_t arg1i;
	uint8_t valid;

	while(uart_available()) {
		c = uart_get();
	//while (1) { // ***debug***
		//c = tst[tsti]; // ***debug***
		//tsti++; // ***debug***
		// end of command: it is ready to be processed
		if (c == ENDCHAR) {
			buf[index] = 0; // marks end of string
			valid = 1;
			// checks for command validity
			if (index < 1) valid = 0;
			cmd = readByte(buf, &valid);
			// if command is a hex number, tries to execute it
			if (valid) {
				switch (cmd) {
				// POWER & DIRECTION COMMANDS
				case 0x01: // set power of left motor
					arg1 = readByte(&buf[2], &valid);
					if (valid) setPowerA(LMOTOR, arg1);
					break;
				case 0x02: // set power of right motor
					arg1 = readByte(&buf[2], &valid);
					if (valid) setPowerA(RMOTOR, arg1);
					break;
				case 0x03: // set direction of left motor
					arg1 = readByte(&buf[2], &valid);
					if (valid) setDirection(LMOTOR, arg1);
					break;
				case 0x04: // set direction of left motor
					arg1 = readByte(&buf[2], &valid);
					if (valid) setDirection(RMOTOR, arg1);
					break;
				case 0x05: // set power for both motors
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						setPowerA(LMOTOR, arg1);
						setPowerA(RMOTOR, arg1);
					}
					break;
				// PID COMMANDS
				case 0x10: // toggle PID on/off
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						pidOn = arg1;
					  resetPID();
					}
					break;
				case 0x11: // set desired speed of left motor
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						if (LMOTOR) {
							targetSpeed1 = arg1 * 2;
						} else {
							targetSpeed0 = arg1 * 2;
						}
						resetPID();
					}
					break;
				case 0x12: // set desired speed of right motor
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						if (RMOTOR) {
							targetSpeed1 = arg1 * 2;
						} else {
							targetSpeed0 = arg1 * 2;
						}
						resetPID();
					}
					break;
				case 0x15: // set speed for both motors
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						targetSpeed0 = arg1 * 2;
						targetSpeed1 = arg1 * 2;
						resetPID();
					}
					break;
				case 0x17: // toggle adjX on/off (auto, this one is preferred)
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						slipAdjFOn = arg1;
						resetPID();
					}
					break;
				case 0x18: // toggle adjX on/off (auto, this one is preferred)
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						slipAdjSOn = arg1;
						resetPID();
					}
					break;
				case 0x19: // toggle adjX on/off (manual)
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						adjXOn = arg1;
						resetPID();
					}
					break;
				case 0x20: // reverse direction for both motors
					ldir = (ldir == FORWARD) ? BACKWARD : FORWARD;
					rdir = (rdir == FORWARD) ? BACKWARD : FORWARD;
					setDirection(LMOTOR, ldir);
					setDirection(RMOTOR, rdir);
					break;
				// NAVIGATOR COMMANDS
				case 0x30: // break, put navigator to idle
					navFree(0, 0, ldir, rdir);
					break;
				case 0x31: // navigator in NAV_SYNC mode
					// byte1: speed | byte2H: dirL | byte2L: dirR
					arg1 = readByte(&buf[2], &valid);
					arg2 = readByte(&buf[4], &valid);
					if (valid) navSync(arg1 * 2, arg2 >> 4, arg2 & 0x0f);
					break;
				case 0x32: // navigator in NAV_DIST mode, going forward
					// byte1: speed | byte2-3: distance in mm
					arg1 = readByte(&buf[2], &valid);
					arg1i = readInt(&buf[4], &valid);
					if (valid) navDist(arg1 * 2, FORWARD, arg1i);
					break;
				case 0x33: // navigator in NAV_DIST mode, going backward
					// byte1: speed | byte2-3: distance in mm
					arg1 = readByte(&buf[2], &valid);
					arg1i = readInt(&buf[4], &valid);
					if (valid) navDist(arg1 * 2, BACKWARD, arg1i);
					break;
				case 0x34: // navigator in NAV_FREE mode
					// byte1: speedL | byte2: speedR | byte3H: dirL | byte3L: dirR
					arg1 = readByte(&buf[2], &valid);
					arg2 = readByte(&buf[4], &valid);
					arg3 = readByte(&buf[6], &valid);
					if (valid) navFree(arg1 * 2, arg2 * 2, arg3 >> 4, arg3 & 0x0f);
					break;
				case 0x35: // navigator in NAV_ROT mode, turn CCW by Angle
					// byte1: speed | byte2-3: angle in degrees (positive ang)
					arg1 = readByte(&buf[2], &valid);
					arg1i = readInt(&buf[4], &valid);
					if (valid) navRot1(arg1 * 2, arg1i, ANG_CCW);
					break;
				case 0x36: // navigator in NAV_ROT mode, turn CW by Angle
					// byte1: speed | byte2-3: angle in degrees (positive ang)
					arg1 = readByte(&buf[2], &valid);
					arg1i = readInt(&buf[4], &valid);
					if (valid) navRot1(arg1 * 2, arg1i, ANG_CW);
					break;
				case 0x37: // navigator in NAV_ROT mode, turn to targer heading
					// byte1: speed | byte2-3: heading in degrees
					arg1 = readByte(&buf[2], &valid);
					arg1i = readInt(&buf[4], &valid);
					if (valid) navRot2(arg1 * 2, arg1i);
					break;
				case 0x41: // read heading (-180 to 180)
					arg1i = readInt(&buf[2], &valid);
					if (valid) heading = arg1i;
					//if (debug3) printf("%d\r\n", heading);
					break;
				case 0x42: // send distance travelled
					sendDist();
					break;
				case 0x43: // toggle rotation angle adjustment
					arg1 = readByte(&buf[2], &valid);
					if (valid) rotAdjOn = arg1;
					break;
				//PID PARAMETERS
				case 0x60: // set kP, kI, kD, kX
					arg1 = readByte(&buf[2], &valid);
					arg2 = readByte(&buf[4], &valid);
					arg3 = readByte(&buf[6], &valid);
					arg4 = readByte(&buf[8], &valid);
					if (valid) {
						kP = arg1;
						kI = arg2;
						kD = arg3;
						kX = arg4;
					}
					break;
				case 0x61: // set kP
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						kP = arg1;
					}
					break;
				case 0x62: // set kI
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						kI = arg1;
					}
					break;
				case 0x63: // set kD
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						kD = arg1;
					}
					break;
				case 0x64: // set errIMax/Min
					arg1i = readInt(&buf[2], &valid);
					if (valid) {
						errIMax = arg1i;
						errIMin = -arg1i;
					}
					break;	
				case 0x65: // set adjustMax/Min
					arg1i = readInt(&buf[2], &valid);
					if (valid) {
						adjustMax = arg1i;
						adjustMin = -arg1i;
					}
					break;
				case 0x66: // set kX
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						kX = arg1;
					}
					break;
				case 0x67: // set xCalibration
					arg1 = readByte(&buf[2], &valid);
					if (valid) {
						xCalibration = arg1;
					}
					break;
				// DEBUG COMMANDS
				case 0x70:
					printParams();
					break;
				case 0x71:
					debug1 = debug1 ? 0 : 1;
					break;
				case 0x72:
					debug2 = debug2 ? 0 : 1;
					break;
				case 0x73:
					debug3 = debug3 ? 0 : 1;
					break;
				case 0x74:
					debug4 = debug4 ? 0 : 1;
					break;
				case 0x75:
					debug5 = debug5 ? 0 : 1;
					break;
				case 0x76:
					debug6 = debug6 ? 0 : 1;
					break;
				case 0x77:
					debug7 = debug7 ? 0 : 1;
					break;
				case 0x79:
					printParams2();
					break;
				}
				// if command doesn't exist, do nothing
			}
			index = 0; // clear command buffer
			//return; // ***debug***
		} else {
			// fill command buffer
			buf[index] = c;
			if (index < 31) index ++;
		}
	}
}


// Convert 2 hex characters into a byte and return it.
// If the hex characters are invalid, set valid to 0.
inline uint8_t readByte(char *buf, uint8_t *valid) {
	*valid = *valid & isHex(buf[0]) & isHex(buf[1]);
	return htoi(buf[0], buf[1]);
}

// Return 16-bit unsigned int. See readByte for more info.
inline uint16_t readUInt(char *buf, uint8_t *valid) {
	*valid = *valid & isHex(buf[0]) & isHex(buf[1]) & isHex(buf[2]) & isHex(buf[3]);
	return htoi(buf[0], buf[1]) << 8 | htoi(buf[2], buf[3]);
}

// Return 16-bit signed int. See readByte for more info.
inline int16_t readInt(char *buf, uint8_t *valid) {
	*valid = *valid & isHex(buf[0]) & isHex(buf[1]) & isHex(buf[2]) & isHex(buf[3]);
	return htoi(buf[0], buf[1]) << 8 | htoi(buf[2], buf[3]);
}

void runPID() {
	int16_t adjust, pwr;
	int16_t errD, errP;
	uint16_t tmp;
	int32_t errX;
		
	// Cross adjustment calculation
	if (adjXOn) {
		cli();
		errX = ( (ticks1 * (xCalibration - 128)) >> 10 ) + ticks1 - ticks0;
		if (rdir) errX = -errX; // reverse correction when going backwards
		sei();
		adjustX = errX * kX >> 8;
		if (adjustX > adjustXMax) adjustX = adjustXMax;
		if (adjustX < adjustXMin) adjustX = adjustXMin;
	} else {
		adjustX = 0;
	}

	// motor0 calculations
	tmp = speed0;
	if (period0) {
		speed0 = 32000 / period0; // speed in ticks/second
	} else {
		speed0 = 0;
	}
	if (targetSpeed0 == 0) {
		power0 = 0;
	} else {
		errP = targetSpeed0 - speed0;
		errI0 += errP;
		if (errI0 > errIMax) errI0 = errIMax;
		if (errI0 < errIMin) errI0 = errIMin;
		// calculate derivative of speed
		errD = speed0 - lastSpeed0[lastSpeedInd0];
		// update last speed
		lastSpeed0[lastSpeedInd0] = tmp;
		lastSpeedInd0++;
		lastSpeedInd0 &= 0x07;

		adjust = (errP * kP) >> 8;
		adjust += ((int32_t)errI0 * kI) >> 10;
		adjust -= (errD * kD) >> 6;
		adjust += adjustX;

		if (adjust > adjustMax) adjust = adjustMax;
		if (adjust < adjustMin) adjust = adjustMin;
		pwr = power0 + adjust;
		if (pwr < 0) {
			power0 = 0;
		} else if (pwr > MAXPOW800) {
			power0 = MAXPOW800;
		} else {
			power0 = pwr;
		}
	}

	setPowerB(0, power0);


	// motor1 calculations
	tmp = speed1;
	if (period1) {
		speed1 = 32000 / period1; // speed in ticks/second
	} else {
		speed1 = 0;
	}
	if (targetSpeed1 == 0) {
		power1 = 0;
	} else {
		errP = targetSpeed1 - speed1;
		errI1 += errP;
		if (errI1 > errIMax) errI1 = errIMax;
		if (errI1 < errIMin) errI1 = errIMin;
		errD = speed1 - lastSpeed1[lastSpeedInd1];
		// update last speed
		lastSpeed1[lastSpeedInd1] = tmp;
		lastSpeedInd1++;
		lastSpeedInd1 &= 0x07;

		adjust = (errP * kP) >> 8;
		adjust += ((int32_t)errI1 * kI) >> 10;
		adjust -= (errD * kD) >> 6;
		adjust -= adjustX;

		if (adjust > adjustMax) adjust = adjustMax;
		if (adjust < adjustMin) adjust = adjustMin;
		pwr = power1 + adjust;
		if (pwr < 0) {
			power1 = 0;
		} else if (pwr > MAXPOW800) {
			power1 = MAXPOW800;
		} else {
			power1 = pwr;
		}
	}

	setPowerB(1, power1);

}

/* function that decides where the robot goes by setting target speed of motors
   Possible commands:
    - go straight continuously - NAV_SYNC
	- travel a given distance - NAV_DIST
	- free mode on each motor (includes free rotation) - NAV_FREE
	- rotate to angle - NAV_ROT;
*/

void navigator() {
	int16_t ticksLeft;
	int16_t headingDiff;
	
	switch(navCom) {
	case NAV_NONE: break; // do nothing
	case NAV_SYNC: break; // do nothing special
	case NAV_DIST:
		cli();
		ticksLeft = n_ticks - ticks0;
		sei();
		// stop if target destination reached (when speed is very slow)
		if (ticksLeft <= 0) {
			targetSpeed0 = 0;
			targetSpeed1 = 0;
			navCom = NAV_DCHK;
		// stop if target destination almost reached (when speed is relatively fast)
		} else if (ticksLeft <= n_stopTicks) {
			if (targetSpeed0 >= n_slowSpeed) {
				targetSpeed0 = 0;
				targetSpeed1 = 0;
				navCom = NAV_DCHK;
			}
		// slow down if getting close to destination
		} else if (ticksLeft <= n_slowTicks) {
			if (targetSpeed0 > n_slowSpeed) {
				targetSpeed0 = n_slowSpeed;
				targetSpeed1 = n_slowSpeed;
			}
		}
		break;
	case NAV_FREE: break; // do nothing special
	case NAV_ROT:
		cli();
		ticksLeft = n_ticks - ticks0;
		sei();
		// stop if target destination reached (when speed is very slow)
		if (ticksLeft <= 0) {
			targetSpeed0 = 0;
			targetSpeed1 = 0;
			n_timer = timer;
			navCom = NAV_RCHK;
		// stop if target destination almost reached (when speed is relatively fast)
		} else if (ticksLeft <= n_stopATicks) {
			if (targetSpeed0 >= n_slowASpeed) {
				targetSpeed0 = 0;
				targetSpeed1 = 0;
				n_timer = timer;
				navCom = NAV_RCHK;
			}
		// slow down if getting close to destination
		} else if (ticksLeft <= n_slowATicks) {
			if (targetSpeed0 > n_slowASpeed) {
				targetSpeed0 = n_slowASpeed;
				targetSpeed1 = n_slowASpeed;
			}
		}
		break;
	// wait for full stop or timeout after displacement
	case NAV_DCHK:
		if ( (!speed0 && !speed1) || ((timer - n_timer) >= n_wait) ) {
			navCom = NAV_NONE;
			sendDone();
		}
		break;
	// check if heading is right after rotation; otherwise rotate again
	case NAV_RCHK: 
		if ( (timer - n_timer) >= n_wait ) {
			headingDiff = adjustAng(n_targetHeading - heading);
			printf("heading diff: %d\r\n", headingDiff);
			if (headingDiff < 0) headingDiff = -headingDiff;
			if (headingDiff <= n_stopAngle || !rotAdjOn) {
				navCom = NAV_NONE;
				sendDone();
				printf("done, target: %d\r\n", n_targetHeading);
			} else {
				printf("again, target: %d\r\n", n_targetHeading);
				navRot2(25, n_targetHeading);
			}
		}
	break;
	}
}

void navSync(int16_t speed, uint8_t dirL, uint8_t dirR) {
	resetPID();
	if (dirL != ldir) setDirection(LMOTOR, dirL);
	if (dirR != rdir) setDirection(RMOTOR, dirR);
	n_targetHeading = heading;
	targetSpeed0 = speed;
	targetSpeed1 = speed;
	slipAdjust();
	navCom = NAV_SYNC;
}

void navDist(int16_t speed, uint8_t dir, int16_t distance) {
	resetPID();
	if (dir != ldir) setDirection(LMOTOR, dir);
	if (dir != rdir) setDirection(RMOTOR, dir);
	n_targetHeading = heading;
	targetSpeed0 = speed;
	targetSpeed1 = speed;
	n_ticks = (int32_t) distance * DISTTOTICKS >> 8;
	slipAdjust();
	navCom = NAV_DIST;
}

void navFree(int16_t speedL, int16_t speedR, uint8_t dirL, uint8_t dirR) {
	resetPID();
	if (dirL != ldir) setDirection(LMOTOR, dirL);
	if (dirR != rdir) setDirection(RMOTOR, dirR);
	adjXOn = 0;
	setTargetSpeed(LMOTOR, speedL);
	setTargetSpeed(RMOTOR, speedR);
	if (speedL || speedR) {
		navCom = NAV_FREE;
	} else {
		navCom = NAV_NONE; // quick way for breaking if speedL = speedR = 0
	}
}

void navRot1(int16_t speed, int16_t angle, uint8_t dir) {
	resetPID();
	if (dir == ANG_CCW) {
		setDirection(LMOTOR, BACKWARD);
		setDirection(RMOTOR, FORWARD);
		n_targetHeading = adjustAng(heading + angle);
	} else {
		setDirection(LMOTOR, FORWARD);
		setDirection(RMOTOR, BACKWARD);
		n_targetHeading = adjustAng(heading - angle);
	}
	
	adjXOn = 0;
	targetSpeed0 = speed;
	targetSpeed1 = speed;
	n_ticks = ((int32_t) angle * ANGLETOTICKS) >> 8;
	navCom = NAV_ROT;
}

void navRot2(int16_t speed, int16_t tHeading) {
	int16_t angle;
	uint8_t dir;
	
	resetPID();
	dir = ANG_CCW;
	n_targetHeading = adjustAng(tHeading);
	angle = deltaAng(heading, tHeading, dir);
	if (angle < 0) {
		angle = -angle;
		dir = ANG_CW;
	}
	
	if (dir == ANG_CCW) {
		setDirection(LMOTOR, BACKWARD);
		setDirection(RMOTOR, FORWARD);
	} else {
		setDirection(LMOTOR, FORWARD);
		setDirection(RMOTOR, BACKWARD);
	}
	adjXOn = 0;
	targetSpeed0 = speed;
	targetSpeed1 = speed;
	n_ticks = (int32_t) angle * ANGLETOTICKS >> 8;
	navCom = NAV_ROT;
}

// returns angle between endAng and startAng when rotating in a given direction
// return value from -179 to 180
// direction: ANG_CW or ANG_CCW
int16_t deltaAng(int16_t startAng, int16_t endAng, uint8_t direction) {
	int16_t angle;
	if (direction == ANG_CCW) {
		angle = endAng - startAng;
	} else {
		angle = startAng - endAng;
	}
	while (angle <= -180) angle += 360;
	while (angle > 180) angle -= 360;
	return angle;
}

// maps any angle to be from -179 to 180
int16_t adjustAng(int16_t angle) {
	while (angle <= -180) angle += 360;
	while (angle > 180) angle -= 360;
	return angle;
}

// adjustment for slippage
void slipAdjust() {
	adjXOn = 0;
	if (angleWithin(n_targetHeading, 90, 20)) {
		if (slipAdjFOn) {
			if (!rdir) {
				n_ticks = ( (int32_t) n_ticks * SLIPADJ90 ) >> 8;
			} else {
				n_ticks = ( (int32_t) n_ticks * SLIPADJ270 ) >> 8;
			}
		}
	} else if (angleWithin(n_targetHeading, -90, 20)) {
		if (slipAdjFOn) {
			if (!rdir) {
				n_ticks = ( (int32_t) n_ticks * SLIPADJ270 ) >> 8;
			} else {
				n_ticks = ( (int32_t) n_ticks * SLIPADJ90 ) >> 8;
			}
		}
	} else if (angleWithin(n_targetHeading, 0, 20)) {
		if (slipAdjSOn) {
			xCalibration = SLIPADJ0;
			adjXOn = 1;
		}
	} else if (angleWithin(n_targetHeading, 180, 20)) {
		if (slipAdjSOn) {
			xCalibration = SLIPADJ180;
			adjXOn = 1;
		}
	}
}

// checks if angle1 is within maxDelta from angle2
uint8_t angleWithin(int16_t angle1, int16_t angle2, int16_t maxDelta) {
	int16_t delta;
	delta = deltaAng(angle1, angle2, ANG_CCW);
	if (delta < 0) delta = -delta;
	if (delta <= maxDelta) {
		return 1;
	} else {
		return 0;
	}
}

// sends done acknowledgement
void sendDone() {
	uart_put('D', NULL);
	uart_put('\r', NULL);
}

// sends travelled distance in mm for each track
void sendDist() {
	int16_t ticks0_cached, ticks1_cached;
	int16_t dist0, dist1;
	cli();
	ticks0_cached = ticks0;
	ticks1_cached = ticks1;
	sei();
	dist0 = (int32_t) ticks0_cached * TICKSTODIST >> 8;
	dist1 = (int32_t) ticks1_cached * TICKSTODIST >> 8;
	printf("T%04x%04x\r", dist0, dist1);
}

void resetPID () {
	uint8_t i;
	uint16_t period0_cached, period1_cached;
	
	cli();

	errI0 = 0;
	errI1 = 0;
	ticks0 = 0;
	ticks1 = 0;
	
	period0_cached = period0;
	period1_cached = period1;
	
	sei();
	
	if (period0_cached) {
		speed0 = 32000 / period0_cached;
	} else {
		speed0 = 0;
	}
	
	if (period1_cached) {
		speed1 = 32000 / period1_cached;
	} else {
		speed1 = 0;
	}

	for (i=0; i<8;i++) {
		lastSpeed0[i] = speed0;
		lastSpeed1[i] = speed1;
	}
}

void printParams() {
	printf_P(PSTR("PIDX:\t%u\t%u\t%u\t%u\r\n"), kP, kI, kD, kX);
	printf_P(PSTR("errI, adj, adjX, xCal:\t%d\t%d\t%d\t%u\r\n"), errIMax, adjustMax, adjustXMax, xCalibration);
}

void printParams2() {
	printf_P(PSTR("slipAdjF, slipAdjS, adjXOn, adjustX:\t%d\t%d\t%d\r\n"), slipAdjFOn, slipAdjSOn, adjXOn, adjustX);
}
