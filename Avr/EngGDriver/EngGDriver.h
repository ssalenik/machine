#ifndef ENGGDRIVER_H
#define ENGGDRIVER_H

#include <avr/io.h>
// include the defines that are common to EngGDriver and EngGMain
#include "../common/common.h"

/* VERSION */
#define VERSION "2.5"

/* ==========  Macros  ========== */
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

/* ==========  Defines  ========== */

/* --- Communication constants --- */
#define ENDCHAR '\r'
#define CPUCHAR '>'
#define MAINCHAR '@'
#define SPEEDMULT 16

/* --- Motor driver constants --- */
    // This is the max value to be placed in the OC1x register
#define MAXTIMER 4000       // 1600 at 8Mhz, 4000 at 20Mhz
    // Those are the max power cap values for different setPower functions
#define POWERCAP 100        // caps the maximum power in %
#define MAXPOWER (MAXTIMER / 100 * POWERCAP)
#define MAXPOW100 POWERCAP
#define MAXPOW800 (POWERCAP * 8)

/* --- Speed calculation related --- */
#define SPEED_CALC_PERIOD 10 // Period of PID and Speed calculations in ms
#define NULL_PERIOD_THRESHOLD (16 * 20) // speeds of less than about 5.5 mm/s are deemed as 0 mm/s
#define TIMER20_FREQ 20000  // Frequency of timer16 (i.e. 20 kHz)

/* --- Motor PINOUT --- */
 
/* for LMOTOR and RMOTOR, values are :
 *  - 0 for OC1A
 *  - 1 for OC1B
 * for M0IN1, M0IN2, M1IN1, M1IN2, pins are:
 *  - M0IN1 & M01N2 are direction pins for motor 0 / OC1A
 *  - M1IN1 & M11N2 are direction pins for motor 1 / OC1B
 *  - PORTM0IN1 and M0IN1 is the PORT and pin combination for motor 0 input 1
*/

#define LMOTOR 0            // Left Motor is Motor 0
#define RMOTOR 1            // Right Motor is Motor 1

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

#define DDRARR  DDRC
#define PORTARR PORTC
#define ARRF 2
#define ARRB 3

#define FORWARD 0           // High-Low
#define BACKWARD 1          // Low-High 
#define BRAKE 2             // High-High
#define NEUTRAL 3           // Low-Low

/* --- Odometer --- */
    // conversions between ticks and distance 
    // basically, 1 tick = 0.11 mm
#define DISTTOTICKS_L 2497  // DIST in mm -> TICKS / 256 (Left Motor)
#define TICKSTODIST_L 105   // TICKS -> DIST in 1/1024th of a mm (Left Motor)
#define DISTTOTICKS_R 2497  // DIST in mm -> TICKS / 256 (Right Motor)
#define TICKSTODIST_R 105   // TICKS -> DIST in 1/1024th of a mm (Right Motor)

/* --- Track Sensor and pos correction --- */
#define MAX_CORR_ERROR 50           // maximum allowable pos correction (in mm)
#define POS_SENSOR_THR 300          // analog sensor plank detection threshold: no plank < 300 < plank
        
/* --- Navigation --- */        
#define RAMPDOWN_MIN_SPEED 320      // minimum speed set for ramp-down
#define RAMPDOWN_REF_DIST 15        // distance at which the speed is capped at RAMPDOWN_MIN_SPEED
#define RAMPDOWN_STOP_DIST 3        // distance at which to stop the robot completely
#define RAMPDOWN_RATE 20            // target speed = RAMPDOWN_RATE * (dist left - RAMPDOWN_REF_DIST)
                                    // slows down until RAMPDOWN_MIN_SPEED
#define RAMPDOWN_CHECK_DIST 200     // distance at which to check for rampdown
                                    

/* OBOSLETE> */
    // basically, 1 ticks = 0.64 degrees
#define ANGLETOTICKS 400        // OBSOLETE
    // Slip adjustment  
#define SLIPADJ0 136            // OBSOLETE
#define SLIPADJ90 280           // OBSOLETE
#define SLIPADJ180 120          // OBSOLETE
#define SLIPADJ270 248          // OBSOLETE
/* <OBSOLETE */

#define ANG_CW 0
#define ANG_CCW 1
/* -------- */


/* ======================== */
/* --- EngGDriver.c: Main function, system timer --- */
    // init all modules and system timer
void initAll();
void initADC();
void initTimer0();
int32_t uptime();   // return the value of timer (interrupt safe)
    // Interrupt vector of EngGDriver.c:
// --> ISR(TIMER0_COMPA_vect)

/* --- motors.c: Motor control, encoders, PID --- */
    // initialize hardware for motors & encoders
void initMotorPins();
void initPWM1();
void initEncoders();
void initTachoVariables();
    // set power and direction
void setPower(uint8_t motor, uint16_t power);
void setPower100(uint8_t motor, uint8_t power100);
void setPower800(uint8_t motor, uint16_t power800);
void setDirection(uint8_t motor, uint8_t direction);
void updateArrows();
    // calculate speed and acceleration and run PID
void calculateSpeeds();
void resetPID();
void runPID();
    // Interrupt vectors of motor.c are:
// --> ISR(INT0_vect)
// --> ISR(INT1_vect)

/* --- communication.c: communication and com related functions */
    // com dispatcher and its helper functions
void readCommand();
inline uint8_t readByte(char *buf, uint8_t *valid);
inline int16_t readInt(char *buf, uint8_t *valid);
inline uint16_t readUInt(char *buf, uint8_t *valid);
    // functions called by dispatcher
void sendDist();
void printParams();
void printParams2();

void sendDone(char dest);
void printDirection(char dest);
void printTicks(char dest);
void printSpeed(char dest);
void printAccel(char dest);
void printAbsDist(char dest);
void printRelDist(char dest);
void printSensors(char dest);


/* --- odometer.c: Odometer and Position Correction --- */
    // Odometer (convert ticks to position: absolute and relative)
void resetOdometer();
void runOdometer();
void updateRelativePos();
void setOdometerTo(int16_t posL, int16_t posR);
    // position correction (read track sensors and do correction)
void resetPosCorrection();
void positionCorrection();
void readTrackSensors();
    // helper functions
int16_t absoluteToRelativePos_L(int16_t absPosL, int8_t *pTransition);
int16_t absoluteToRelativePos_R(int16_t absPosR, int8_t *pTransition);
int16_t relativeToAbsolutePos_L(int16_t relPosL, uint8_t transition);
int16_t relativeToAbsolutePos_R(int16_t relPosR, uint8_t transition);

/* --- navigation.c: all navigation related stuff. To be modified --- */
void navigator();
int16_t calculateTargetSpeed(int16_t dist_Left, int16_t curTargetSpeed);
int16_t getDistLeft(uint8_t dir, int16_t target, int16_t current);
void navDest(int16_t speedL, int16_t speedR, int16_t posL, int16_t posR);
void navDestRel(int16_t speedL, int16_t speedR, uint8_t transL, int16_t offsetL, 
    uint8_t transR, int16_t offsetR);
void setTargetSpeed(uint8_t motor, int16_t speed);

// OBSOLETE:
void navSync(int16_t speed, uint8_t dirL, uint8_t dirR);
void navDist(int16_t speed, uint8_t dir, int16_t distance);
void navFree(int16_t speedL, int16_t speedR, uint8_t dirL, uint8_t dirR);
void navRot1(int16_t speed, int16_t angle, uint8_t dir);
void navRot2(int16_t speed, int16_t tHeading);
// /OBSOLETE:


int16_t deltaAng(int16_t startAng, int16_t endAng, uint8_t direction);
int16_t adjustAng(int16_t angle);
uint8_t angleWithin(int16_t angle1, int16_t angle2, int16_t maxDelta);

#endif //ENGGDRIVER_H
