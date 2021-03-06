
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
#include "variables.c"          // contains all the public variables
#include "delay.c"
#include "adc.c"
#include "motors.c"
#include "communication.c"
#include "odometer.c"
#include "navigation.c"
/* --------------*/

/**
 * Main loop.
 */
int main(void) {
    int32_t lastSpeedCalc = 0;
    int32_t lastDebug = 0;
    int32_t lastDebug2 = 0;
    
    // initalize all modules
    initAll();
    
    // enable interrupts (i.e timers start from here)
    sei();
    
    // The LOOP
    while(1) {
        // read serial port
        readCommand();          // communication.c
        
        // apply speed calculation and run odometer
        if (uptime() >= lastSpeedCalc + SPEED_CALC_PERIOD) {
            lastSpeedCalc += SPEED_CALC_PERIOD;
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
        
        // print debug loop 2
        if (uptime() >= (lastDebug2 + 20)) {
            lastDebug2 = uptime();
            if (debug6) printf("%d, %d\r\n", p_LsensVal, p_RsensVal);
        }
        
        // print debug
        if ((debugPeriod) && uptime() >= (lastDebug + debugPeriod)) {
            lastDebug += debugPeriod;
            //if (debug1) printf(">21%02x\r", speed0 >> 4);
            //if (debug1) printf(">21%04x\r>22%04x\r", readADC(0), readADC(1));
            //if (debug2) printf("%ld\t%ld\r\n", timer, timer20);
            //if (debug2) printf_P(PSTR("%d\t%d\r\n"), p_L, p_R);
            if (debug3) printf_P(PSTR("%d, %d / %d, %d / %d, %d / %u, %u\r\n"),
                p_Ltrans, p_Lrel, p_Rtrans, p_Rrel, p_Lerr, p_Rerr, posCorrLeftFailed, posCorrRightFailed);
            //if (debug4) printf_P(PSTR("%ld\t%ld\r\n"), ticks0, ticks1);
            //if (debug5) printSpeed(CPUCHAR);
            //if (debug5) printf_P(PSTR("%d\t%d\r\n"), speed0, speed1);
            //if (debug6) printf_P(PSTR("%d\t%d\r\n"), accel0, accel1);
            if (debug7) printf_P(PSTR("%u\t%u\r\n"), power0, power1);
            // debug for GUI
            printTicks(CPUCHAR);
            printSpeed(CPUCHAR);
            printAccel(CPUCHAR);
            printAbsDist(CPUCHAR);
            printRelDist(CPUCHAR);
            printSensors(CPUCHAR);
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
    // total frequency is 8kHz at 20Mhz clock (20M / 125 / 8 = 20k)
    TIMSK0 |= 1 << OCIE0A;  // enable compare A interupt
}

/**
 * Timer0 COMPARE A interrupt used for system timer
 */
ISR(TIMER0_COMPA_vect) {
    timer20++;
    timerPrescaler++;
    if (timerPrescaler == 20) {
        timer++;
        timerPrescaler = 0;
    }
}

int32_t uptime() {
    int32_t timer_cached;
    cli();
    timer_cached = timer;
    sei();
    return timer_cached;
}

