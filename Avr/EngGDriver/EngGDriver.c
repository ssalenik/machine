
/* VERSION */
#define VERSION "2.0"

/* Includes */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "uart.h"
#include "hex.h"

#include "EngGDriver.h"
/* --------------*/


/* ======== Variables ======== */

/* --- Timers ---*/
volatile int32_t timer16 = 0; 			// timer in 1/16nd of a millisecond
volatile uint8_t timerPrescaler = 0; 	// used to increment timer
volatile int32_t timer = 0; 			// timer in milliseconds
/* --------------*/

/* --- Speed and Acceleration --- */
volatile int32_t ticks0 = 0;			// encoder values for motor0 (i.e. tachometer)
volatile int32_t ticks1 = 0;			// encoder values for motor1 (i.e. tachometer)
	// Timestamps of previous encoder interrupts:
volatile int32_t int0Time0 = 0;			// latest timestamp from encoder 0 interrupt
volatile int32_t int0Time1 = 0;			// previous timestamp from encoder 0 interrupt
volatile int32_t int1Time0 = 0; 		// latest timestamp from encoder 1 interrupt
volatile int32_t int1Time1 = 0; 		// previous timestamp from encoder 1 interrupt
int32_t int0TimeCS = 0, int1TimeCS = 0; // encoder timestamps when last called calculateSpeeds();
int32_t ticks0CS = 0, ticks1CS = 0; 	// ticks# value when int#TimeCS was recorded
volatile int8_t int0dir = FORWARD; 		// latest recorded direction of rotation
volatile int8_t int1dir = FORWARD; 		// latest recorded direction of rotation
	// Note: Speed is in ticks / sec, where 1 tick = 0.110 mm
int16_t speed0 = 0, speed1 = 0; 		// speed of motor in ticks / sec
int16_t accel0 = 0, accel1 = 0; 		// acceleration of motor in ticks / sec^2
int16_t lastSpeed0[8], lastSpeed1[8]; 	// values used by calculateSpeeds() for acceleration
int16_t lastSpeedInd = 0; 				// index for the current value in lastSpeed buffer
int32_t lastSpeedCalc = 0; 				// SpeedCalculation and PID timer
/* --------------*/

/* --- PID parameters --- */
uint8_t kP = 32;						// P Constant
uint8_t kI = 16;						// I Constant
uint8_t kD = 64;						// D Constant
uint8_t kX = 64;						// Cross dependency between both motor displacements
int16_t errIMax = 800;					// Max Integer value
int16_t errIMin = -800;			
int16_t adjustMax = 30;					// Max power adjustment factor
int16_t adjustMin = -20;			
int16_t adjustXMax = 50;				// Max power adjustment from kX
int16_t adjustXMin = -50; 			
uint8_t xCalibration = 128;				// calibrates motor1 to motor0 using ticks. 128 for no calibration
/* --------------*/

/* --- Motor Control and PID variables --- */
uint8_t pidOn = 0; 						// turns PID on or off
uint8_t ldir = FORWARD;					// direction in which left Motor is set to rotate
uint8_t rdir = FORWARD;					// direction in which left Motor is set to rotate
uint16_t targetSpeed0 = 0;				// target speed of motor 0
uint16_t targetSpeed1 = 0;				// target speed of motor 1
int16_t errI0 = 0, errI1 = 0;			// Accumulator for the PID I part
uint16_t power0 = 0, power1 = 0;		// power (0 to 100 * 256) applied to each motor
uint8_t adjXOn = 0; 					// enable or disable cross adjustment
int16_t adjustX = 0;					// current cross adjustment
/* --------------*/

// TODO: Change all this navigation stuff
/* --- Navigation variables --- */
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
int16_t heading = 0; 					// current heading of the robot as recieved from master
int16_t n_ticks = 0; 					// target distance in ticks to travel by navigator when in mode NAV_DIST
int16_t n_slowTicks = 100; 				// distance in ticks before slowing down to n_slowSpeed if targetSpeed > n_slowSpeed
int16_t n_stopTicks = 8; 				// distance in ticks when to stop if targetSpeed = n_slowSpeed
int16_t n_slowSpeed = 50;
int16_t n_targetHeading = 0; 			// target heading to travel by navigator when in mode NAV_ROT
//int16_t n_angleTarget = 0; 			// target angle to travel by navigator when in mode NAV_ROT
//int16_t n_angle = 0; 					// angle travelled in mode NAV_ROT
//int16_t n_lastHeading = 0; 			// last heading reading by navigator
//int16_t n_slowAngle = 10; 			// angle from target heading before slowing down if targetSpeed > n_slowASpeed;
int16_t n_stopAngle = 2; 				// angle from target heading when to stop if targetSpeed = n_slowASpeed
int16_t n_slowATicks = 50;
int16_t n_stopATicks = 8;
int16_t n_slowASpeed = 35;
int16_t n_wait = 500; 					// wait in ms before checking that heading is same as target
int32_t n_timer = 0;
uint8_t n_rot_dir = ANG_CW;  			// direction of rotation for NAV_ROT
uint8_t slipAdjFOn = 0;
uint8_t slipAdjSOn = 0;
uint8_t rotAdjOn = 1;

/* --- Debug --- */
uint8_t debug1 = 0;
uint8_t debug2 = 0;
uint8_t debug3 = 0;
uint8_t debug4 = 0;
uint8_t debug5 = 0;
uint8_t debug6 = 1;
uint8_t debug7 = 0;
uint16_t d_count1 = 0;
void debugF1();
/* --------------*/

/* This statement allows printf to work with serial com
   for every character sent to stream uart_stdout, uart_put is executed
   later in code, stdout is set to uart_stdout, so that
   printf writes to the stream uart_stdout automatically */
static FILE uart_stdout = FDEV_SETUP_STREAM(uart_put, NULL, _FDEV_SETUP_WRITE);

/* ======================== */


int main(void) {
	int32_t lastTime = 0;

	// redirect printf output to serial port
	stdout = &uart_stdout;

	initVariables();
	
	// *** TEMP DEBUG ***
	//calculateSpeeds();
	// /DEBUG
	
	resetPID();

	uart_init();

	initTimer0();
	initPWM1();
	initADC();
	initMotorPins();
	sbi(DDRC, 3);
	sbi(DDRC, 4);
	
	EICRA = 0x0f;
	EIMSK = 0x03;

	setDirection(LMOTOR, FORWARD);
	setDirection(RMOTOR, FORWARD);
	setPowerA(LMOTOR, 20);
	setPowerA(RMOTOR, 20);

	sei(); // enable interrupts
 
	while(1) {
		readCommand();
		
		if (timer - lastSpeedCalc >= SPEED_CALC_PERIOD) {
			lastSpeedCalc = timer;
			calculateSpeeds();
			if (pidOn) {	
				//navigator();
				runPID();
			}
		}
			
		if (timer >= (lastTime + 100)) {
			lastTime += 100;
			if (debug7) printf("%u\t%u\r\n", power0, power1);
			if (debug6) printf("%d\t%d\t%d\t%d\r\n", speed0, speed1, accel0, accel1);
			if (debug5) printf ("%ld\t%ld\r\n", ticks0, ticks1);
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
			sbi(PORTM1IN1, M1IN1);
			cbi(PORTM1IN2, M1IN2);
			break;
		case BACKWARD:
			cbi(PORTM1IN1, M1IN1);
			sbi(PORTM1IN2, M1IN2);
			break;
		case BRAKE:
			sbi(PORTM1IN1, M1IN1);
			sbi(PORTM1IN2, M1IN2);
			break;
		default:
			cbi(PORTM1IN1, M1IN1);
			cbi(PORTM1IN2, M1IN2);
		}
	} else {
		switch(direction) {
		case FORWARD:
			sbi(PORTM0IN1, M0IN1);
			cbi(PORTM0IN2, M0IN2);
			break;
		case BACKWARD:
			cbi(PORTM0IN1, M0IN1);
			sbi(PORTM0IN2, M0IN2);
			break;
		case BRAKE:
			sbi(PORTM0IN1, M0IN1);
			sbi(PORTM0IN2, M0IN2);
			break;
		default:
			cbi(PORTM0IN1, M0IN1);
			cbi(PORTM0IN2, M0IN2);
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

// Configure timer0 to generate interrupts at 16KHz
void initTimer0() {
	TCCR0A = 0b00000010;	// set timer to CTC mode	
	TCCR0B = 0b00000001;	// set clock prescaler to 8
	OCR0A = 124;			// set to 124 + 1 timer ticks per interrupt
	// total frequency is 8kHz at 20Mhz clock (20M / 125 / 8 = 16k)
	TIMSK0 |= 1 << OCIE0A;	// enable compare A interupt
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
	// configure ADC module
	ADMUX |= 1 << REFS0;  // set ref. voltage to AVcc
	ADMUX |= 1 << ADLAR;  // left justify output	
	
	// Note: ADC channel is set in bits 3..0 of ADMUX. 0000 for chan0
	ADCSRA |= 7;		  // set ADC clock prescaler to 64
	ADCSRA |= 1 << ADEN;  // enable ADC
}

void initMotorPins() {
	//DDRB |=  0b00111001;	// set pins 0, 3, 4, 5 as output in PORT B
	sbi(DDRM0IN1, M0IN1);
	sbi(DDRM0IN2, M0IN2);
	sbi(DDRM1IN1, M1IN1);
	sbi(DDRM1IN2, M1IN2);
	/*cbi(M0IN1);
	cbi(M0IN2);
	cbi(M1IN1);
	cbi(M1IN2);*/
	//PORTB &= 0b11000110;	// set the values of those pins to 0
}

uint8_t readADC(uint8_t channel) {
	channel &= 0x0f;
	ADMUX &= 0xf0;
	ADMUX |= channel; 		// choose channel
	ADCSRA |= 1 << ADSC;	// start conversion	
	while (ADCSRA & (1 << ADSC)); // wait for convertion to finish
	return ADCH;
}

// Interrupt for Motor 0 encoder
ISR(INT0_vect) {
	uint8_t dir;
	// check direction of rotation
	dir = PIND & (1 << 4);
	// TODO: confirm which direction is which
	if (dir) {
		int0dir = FORWARD;
		ticks0++;
	} else {
		int0dir = BACKWARD;
		ticks0--;
	}
	
	// update the timestamp of the latest tick and the one before
	int0Time1 = int0Time0;
	int0Time0 = timer16;
	
}

// Interrupt for Motor 1 encoder
ISR(INT1_vect) {
	uint8_t dir;
	// check direction of rotation
	dir = PINB & (1 << 7);
	// TODO: confirm which direction is which
	if (dir) {
		int1dir = FORWARD;
		ticks1++;
	} else {
		int1dir = BACKWARD;
		ticks1--;
	}
	
	// update the timestamp of the latest tick and the one before
	int1Time1 = int1Time0;
	int1Time0 = timer16;
}


// Timer0 COMPARE A interrupt used for timer
ISR(TIMER0_COMPA_vect) {	
	timer16++;
	timerPrescaler++;
	if (!(timerPrescaler & 0x0f)) timer ++;
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
	*valid = *valid & isHex(buf[0]) & isHex(buf[1]) & 
		isHex(buf[2]) & isHex(buf[3]);
	return htoi(buf[0], buf[1]) << 8 | htoi(buf[2], buf[3]);
}

// Return 16-bit signed int. See readByte for more info.
inline int16_t readInt(char *buf, uint8_t *valid) {
	*valid = *valid & isHex(buf[0]) & isHex(buf[1])	& 
		isHex(buf[2]) & isHex(buf[3]);
	return htoi(buf[0], buf[1]) << 8 | htoi(buf[2], buf[3]);
}

void runPID() {
	int16_t adjust, pwr;
	int16_t errD, errP;
	//int32_t errX;
		
	/* CROSS ADJUSTMENT CALCULATIONS: Makes sure both motors travel same dist */
	/*if (adjXOn) {
		cli();
		errX = ( (ticks1 * (xCalibration - 128)) >> 10 ) + ticks1 - ticks0;
		if (rdir) errX = -errX; // reverse correction when going backwards
		sei();
		adjustX = errX * kX >> 8;
		if (adjustX > adjustXMax) adjustX = adjustXMax;
		if (adjustX < adjustXMin) adjustX = adjustXMin;
	} else {
		adjustX = 0;
	}*/

	/* --- Motor0 CALCULATIONS --- */
	if (targetSpeed0 == 0) {
		power0 = 0;
	} else {
		// The P parameter is the error of current speed vs target speed
		errP = targetSpeed0 - speed0;
		// The I parameter is the intergral of P errors. 
		// It must be capped to reduce oscillations
		errI0 += errP;
		if (errI0 > errIMax) errI0 = errIMax;
		if (errI0 < errIMin) errI0 = errIMin;
		// The D parameter is equal to the acceleration
		errD = accel0;

		// Perform a weighted addition of errors to calculate power adjustment
		adjust = ((int32_t)errP * kP) >> 8;
		adjust += ((int32_t)errI0 * kI) >> 10;
		adjust -= ((int32_t)errD * kD) >> 8;
		//adjust += adjustX;

		// Since power is unsigned, and 'adjust' sees power as signed,
		// we must correct its sign
		if (targetSpeed0 < 0) adjust =-adjust;
		
		// Cap the power adjustment to reduce oscillations
		if (adjust > adjustMax) adjust = adjustMax;
		if (adjust < adjustMin) adjust = adjustMin;
		
		// Adjust power and cap it to avoid burning motors or overflowing PWM
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


	/* --- Motor1 CALCULATIONS: Identical to Motor0 --- */
	if (targetSpeed1 == 0) {
		power1 = 0;
	} else {
		errP = targetSpeed1 - speed1;
		errI1 += errP;
		if (errI1 > errIMax) errI1 = errIMax;
		if (errI1 < errIMin) errI1 = errIMin;
		errD = accel1;

		adjust = ((int32_t)errP * kP) >> 8;
		adjust += ((int32_t)errI1 * kI) >> 10;
		adjust -= ((int32_t)errD * kD) >> 8;
		//adjust += adjustX;
		
		if (targetSpeed1 < 0) adjust =-adjust;
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
	
	errI0 = 0;
	errI1 = 0;
	
}

void printParams() {
	printf_P(PSTR("PIDX:\t%u\t%u\t%u\t%u\r\n"), kP, kI, kD, kX);
	printf_P(PSTR("errI, adj, adjX, xCal:\t%d\t%d\t%d\t%u\r\n"), 
		errIMax, adjustMax, adjustXMax, xCalibration);
}

void printParams2() {
	printf_P(PSTR("slipAdjF, slipAdjS, adjXOn, adjustX:\t%d\t%d\t%d\r\n"), 
		slipAdjFOn, slipAdjSOn, adjXOn, adjustX);
}

void calculateSpeeds() {
	int32_t int0Time0_cached, int0Time1_cached;
	int32_t int1Time0_cached, int1Time1_cached;
	int32_t ticks0_cached, ticks1_cached;
	int8_t int0dir_cached, int1dir_cached;
	int32_t timer16_cached;
	
	
	// cache volatile variables
	cli();
	int0Time0_cached = int0Time0;
	int0Time1_cached = int0Time1;
	int1Time0_cached = int1Time0;
	int1Time1_cached = int1Time1;
	ticks0_cached = ticks0;
	ticks1_cached = ticks1;
	int0dir_cached = int0dir;
	int1dir_cached = int1dir;
	timer16_cached = timer16;
	sei();
	
	/* CALCULATE SPEED
	 * There are 3 cases:
	 * if speed is slow, ticks# == ticks#CS, thus
	 *   simply use the timestamps of the last 2 crosses.
	 * if speed is very slow or null, ticks# == ticks#CS
	 *   and (timer16 - int#Time0) > NULL_PERIOD_THRESHOLD,
	 *   then just set speed to 0.
	 * if speed is fast enough, then ticks# != ticks#CS
	 */
	
	if (ticks0_cached == ticks0CS) {
		// speed is slow
		if ((timer16_cached - int0Time0_cached) > NULL_PERIOD_THRESHOLD) {
			// speed is very slow or null
			speed0 = 0;
		} else {
			// calculate slow speed using last 2 ticks' timestamps
			speed0 = TIMER16_FREQ / (int0Time0_cached - int0Time1_cached);
			if (int0dir_cached == BACKWARD) speed0 = -speed0;
		}
	} else {
		// speed is faster
		speed0 = ((long)TIMER16_FREQ * (ticks0_cached - ticks0CS)) /
			(int0Time0_cached - int0TimeCS);
	}
	
	if (ticks1_cached == ticks1CS) {
		// speed is slow
		if ((timer16_cached - int1Time0_cached) > NULL_PERIOD_THRESHOLD) {
			// speed is very slow or null
			speed1 = 0;
		} else {
			// calculate slow speed using last 2 ticks' timestamps
			speed1 = TIMER16_FREQ / (int1Time0_cached - int1Time1_cached);
			if (int1dir_cached == BACKWARD) speed1 = -speed1;
		}
	} else {
		// speed is faster
		speed1 = ((long)TIMER16_FREQ * (ticks1_cached - ticks1CS)) /
			(int1Time0_cached - int1TimeCS);
	}
	
	// TODO: cap speeds
	
	/* CALCULATE ACCELERATION
	 * Since the acceleration is in ticks / s ^ 2 and is calculated 
	 *   from variations in speed in a timelapse, in order to obtain
	 *   a better resolution for the value, we must increase the timelapse
	 *   by latching 8 previous values of speed. This comes at the cost
	 *   of temporal resolution. (Heisenberg here... well not really but :) )
	 */
	
	accel0 = ((long)(speed0 - lastSpeed0[lastSpeedInd]) * 1000) / 
		(8 * SPEED_CALC_PERIOD);
	accel1 = ((long)(speed1 - lastSpeed1[lastSpeedInd]) * 1000) / 
		(8 * SPEED_CALC_PERIOD);
	// update previous speeds
	lastSpeed0[lastSpeedInd] = speed0;
	lastSpeed1[lastSpeedInd] = speed1;
	lastSpeedInd++;
	lastSpeedInd &= (8 - 1);
	 
	// update latest tick timestamps recorded in calculateSpeeds()
	int0TimeCS = int0Time0_cached;
	int1TimeCS = int1Time0_cached;
	ticks0CS = ticks0_cached;
	ticks1CS = ticks1_cached;
}

// initialize misc variables
void initVariables() {
	uint8_t i = 0;
	for (i=0; i<8;i++) {
		lastSpeed0[i] = 0;
		lastSpeed1[i] = 0;
	}
	
	// *** TEMP DEBUG ***
	/*int0Time0 = 168;
	int1Time0 = 167;
	timer16 = 170;
	ticks0 = 20;
	ticks1 = 25;
	ticks0CS = 10;
	ticks1CS = 15;
	int0TimeCS = 6;
	int1TimeCS = 9;*/
	// /DEBUG
}
