
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

/* --- insert modules --- */
#include "delay.c"
#include "adc.c"
#include "variables.c"          // contains all the public variables
#include "motors.c"
#include "communication.c"
#include "odometer.c"
#include "navigation.c"
/* --------------*/

/**
 * Main loop.
 */
int main(void) {
    int32_t lastDebug = 0;
    
    // initalize all modules
    initAll();
    
    // enable interrupts
    sei();
    
    // The LOOP
    while(1) {
        // read serial port
        readCommand();          // communication.c
        
        // apply speed calculation and run odometer
        // FIXME: "timer" can get trashed by ISR. Create a function "uptime()" to read timer
        if (timer >= lastSpeedCalc + SPEED_CALC_PERIOD) {
            lastSpeedCalc = timer; // FIXME: lastSpeedCalc += SPEED_CALC_PERIOD; // "timer" can get trashed by ISR
            calculateSpeeds();  // motors.c
            if (pidOn) {
                runPID();       // motors.c
            }
            //if (debug1) printSpeed(CPUCHAR);
            runOdometer();      // odometer.c
            navigator();
        }
        
        // read sensors and apply position correction
        positionCorrection();   // odometer.c
        
        // print debug
        // FIXME: "timer" can get trashed by ISR. Create a function "uptime()" to read timer
        if ((debugPeriod) && timer >= (lastDebug + debugPeriod)) {
            lastDebug += debugPeriod; // FIXME: lastDebug += debugPeriod; // "timer" can get trashed by ISR
            //if (debug1) printf(">21%02x\r", speed0 >> 4);
            //if (debug1) printf(">21%04x\r>22%04x\r", readADC(0), readADC(1));
            if (debug1) printSpeed(CPUCHAR);
            //if (debug2) printf("%ld\t%ld\r\n", timer, timer16);
            if (debug2) printf_P(PSTR("%d\t%d\r\n"), p_L, p_R);
            if (debug3) printf_P(PSTR("%d, %d / %d, %d / %d, %d / %u, %u\r\n"),
                p_Ltrans, p_Lrel, p_Rtrans, p_Rrel, p_Lerr, p_Rerr, posCorrLeftFailed, posCorrRightFailed);
            if (debug4) printf_P(PSTR("%ld\t%ld\r\n"), ticks0, ticks1);
            //if (debug5) printf_P(PSTR("%d\t%d\r\n"), speed0, speed1);
            //if (debug6) printf_P(PSTR("%d\t%d\r\n"), accel0, accel1);
            if (debug7) printf_P(PSTR("%u\t%u\r\n"), power0, power1);
        }
    }
    
    while(1);
}

/**
 * Initialize all modules
 */
void initAll() {
    // initialize ADC
    initADC();
    
    // initialize system timer
    initTimer0();
    
    // initialize serial com
    uart_init();
    // redirect printf output to serial port
    stdout = &uart_stdout;
    
    // motor.c: initialize motor PWM, pins, encoders and PID
    initPWM1();
    initMotorPins();
    initEncoders();
    initTachoVariables();
    resetPID();
    
    // clear motor speed and direction just in case
    setDirection(LMOTOR, FORWARD);
    setDirection(RMOTOR, FORWARD);
    setPower100(LMOTOR, 0);
    setPower100(RMOTOR, 0);
    
    // odometer.c set initial position and position correction
    setOdometerTo(0, 0);
    updateRelativePos();
    resetPosCorrection();
}

/**
 * Configure timer0 to generate interrupts at 16KHz
 */
void initTimer0() {
    TCCR0A = 0b00000010;    // set timer to CTC mode
    TCCR0B = 0b00000010;    // set clock prescaler to 8
    OCR0A = 124;            // set to 124 + 1 timer ticks per interrupt
    // total frequency is 8kHz at 20Mhz clock (20M / 125 / 8 = 16k)
    TIMSK0 |= 1 << OCIE0A;  // enable compare A interupt
    // NOTE: actual frequency with these settings is 20MHz / 8 / (124+1) = 20kHz for "timer16" and 1.25kHz for "timer"
}

/**
 * Timer0 COMPARE A interrupt used for system timer
 */
ISR(TIMER0_COMPA_vect) {
    timer16++;
    timerPrescaler++;
    if (!(timerPrescaler & 0x0f)) timer++;
}

