#ifndef ENGGDRIVER_H
#define ENGGDRIVER_H

#include <avr/io.h>

/*  Defines  */

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define MAXTIMER 4000 // 1600 at 8Mhz, 4000 at 20Mhz
// This is the max value to be placed in the OC1x register
#define POWERCAP 100 // caps the maximum power in %
#define MAXPOWER (MAXTIMER / 100 * POWERCAP)
#define MAXPOW100 POWERCAP
#define MAXPOW800 (POWERCAP * 8)

#define SPEED_CALC_PERIOD 10 // Period of PID and Speed calculations in ms
#define NULL_PERIOD_THRESHOLD (16 * 20) // speeds of less than about 5.5 mm/s are deemed as 0 mm/s
#define TIMER16_FREQ 16000 // Frequency of timer16 (i.e. 16 kHz)

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

#define LMOTOR 0
#define RMOTOR 1

#define M0IN1 5
#define M0IN2 6
#define M1IN1 0
#define M1IN2 7
#define DDRM0IN1 DDRD
#define DDRM0IN2 DDRD
#define DDRM1IN1 DDRB
#define DDRM1IN2 DDRD
#define PORTM0IN1 PORTD
#define PORTM0IN2 PORTD
#define PORTM1IN1 PORTB
#define PORTM1IN2 PORTD

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
void initTimer0();
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
void calculateSpeeds();
void initVariables();

/* -------- */

#endif //ENGGDRIVER_H
