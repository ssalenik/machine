/** 
 * motors.c: 
 * Motor control, encoders, tachometer, speed calculation, PID
 * DO NOT COMPILE, ONLY INCLUDE IN EngGDriver.c
 * For function prototypes and defines, see EngGDriver.h
 * For public variables, see variables.c
 *
 * This module directly controls the power applied to each motor.
 * Currently Motor 0 is Left Motor and Motor 1 is Right Motor,
 * however this is not hardcoded and a translation is always performed.
 * This module also implements the PID motor speed controller.
 */

/** 
 * Interrupt for Motor 0 encoder & tacho counter
 * Used by calculateSpeeds().
 */
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

/** 
 * Interrupt for Motor 1 encoder & tacho counter
 * Used by calculateSpeeds().
 */
ISR(INT1_vect) {
    uint8_t dir;
    // check direction of rotation
    dir = PINB & (1 << 7);
    // TODO: confirm which direction is which
    if (!dir) {
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

 
 /**
  * set motor and arrow pins as output.
  */
 void initMotorPins() {
    sbi(DDRM0IN1, M0IN1);
    sbi(DDRM0IN2, M0IN2);
    sbi(DDRM1IN1, M1IN1);
    sbi(DDRM1IN2, M1IN2);
    sbi(DDRARR, ARRF);
    sbi(DDRARR, ARRB);
}

/**
 * Init PWM on timer 1 for motors.
 */
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

/**
 * Init interrupts for encoders.
 */
void initEncoders() {
    // set external interrupts 0 and 1 to rising edge
    EICRA = 0x0f;
    // enable external interrupts 0 and 1
    sbi(EIMSK, INT0);
    sbi(EIMSK, INT1);
}

/**
 * Init variables used for speed calculation and such.
 */
void initTachoVariables() {
    uint8_t i = 0;
    for (i=0; i<8;i++) {
        lastSpeed0[i] = 0;
        lastSpeed1[i] = 0;
    }
}

/** 
 * Sets the power of the motor. To use with timer1.
 * - 0 for pin PB1 / OC1A
 * - 1 for pin PB2 / OC1B
 * Ranges from 0 to MAXPOWER
 */
void setPower(uint8_t motor, uint16_t power) {
    if (power > MAXPOWER) power = MAXPOWER;
    if (motor) {
        OCR1B = MAXTIMER - power; // opposite phase
        moving1 = power ? 1 : 0;
    } else {
        OCR1A = power;
        moving0 = power ? 1 : 0;
    }
    updateArrows();
}

/**
 * Same as setPower, but ranges from 0-100 for ease of use
 */
void setPower100(uint8_t motor, uint8_t power100) {
    if (power100 > MAXPOW100) power100 = MAXPOW100;
    if (motor) {
        OCR1B = MAXTIMER - (uint16_t)power100 * (MAXTIMER/100); // opposite phase
        moving1 = power100 ? 1 : 0;
    } else {
        OCR1A = (uint16_t)power100 * (MAXTIMER/100);
        moving0 = power100 ? 1 : 0;
    }   
    updateArrows();
} 

/**
 * Same as setPower, but ranges from 0-800 for PID controller
 */
void setPower800(uint8_t motor, uint16_t power800) {
    if (power800 > MAXPOW800) power800 = MAXPOW800;
    if (motor) {
        OCR1B = MAXTIMER - power800 * (MAXTIMER/800); // opposite phase
        moving1 = power800 ? 1 : 0;
    } else {
        OCR1A = power800 * (MAXTIMER/800);
        moving0 = power800 ? 1 : 0;
    }
    updateArrows();
}

/**
 * Define direction in which a motor should turn.
 * direction can be: FORWARD, BACKWARD, BRAKE or NEUTRAL
 * converts from motors 0 and 1 to motors L and R
 */
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

/**
 * Update state of arrow direction indicators depending on
 * last power and direction commands given.
 * Called from setPowerXXX() and setDirection()
 */
void updateArrows() {
    uint8_t arrFwd = 0, arrRev = 0;
    
    if (moving0) {
        if      (ldir == FORWARD) { arrFwd = 1; }
        else if (ldir == BACKWARD) { arrRev = 1; }
    }
    
    if (moving1) {
        if      (rdir == FORWARD) { arrFwd = 1; }
        else if (rdir == BACKWARD) { arrRev = 1; }
    }
    
    if (arrFwd) { sbi(PORTARR, ARRF); }
    else        { cbi(PORTARR, ARRF); }
    
    if (arrRev) { sbi(PORTARR, ARRB); }
    else        { cbi(PORTARR, ARRB); }
}

/**
 * Calculate speed and acceleration of each motor
 * Uses tacho count and the 16 kHz timer to calculate speed
 * See also: INT0 and INT1 interrupt vectors
 */
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

/**
 * Resets PID to initial state.
 * Basially, it only clears the 'I' accumulator.
 */
void resetPID () {
    errI0 = 0;
    errI1 = 0;
}

/**
 * Motor speed control through a PID controller.
 * Variables to set are targetSpeed0 and targetSpeed1.
 * Those variables must be unsigned, in ticks/s.
 * Variables ldir and rdir determine the direction of rotation
 */
void runPID() {
    int16_t adjust, pwr;
    int16_t errD, errP;
    int16_t targetSpeed0s, targetSpeed1s;
    
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
    /* Note: since both targetSpeed and speed are signed and power
     * is unsigned, we need to take some precautions.
     */
    
    // Convert unsigned target speed into signed
    if (RMOTOR) {
        if (ldir == FORWARD) {
            targetSpeed0s = targetSpeed0;
        } else {
            targetSpeed0s = -targetSpeed0;
        }
        if (rdir == FORWARD) {
            targetSpeed1s = targetSpeed1;
        } else {
            targetSpeed1s = -targetSpeed1;
        }
    } else {
        if (rdir == FORWARD) {
            targetSpeed0s = targetSpeed0;
        } else {
            targetSpeed0s = -targetSpeed0;
        }
        if (ldir == FORWARD) {
            targetSpeed1s = targetSpeed1;
        } else {
            targetSpeed1s = -targetSpeed1;
        }
    }
     
    if (targetSpeed0 == 0) {
        power0 = 0;
    } else {
        // The P parameter is the error of current speed vs target speed
        errP = targetSpeed0s - speed0;
        // The I parameter is the intergral of P errors. 
        // It must be capped to reduce oscillations
        errI0 += errP;
        if (targetSpeed0s > 0) {
            if (errI0 > errIMax) errI0 = errIMax;
            if (errI0 < errIMin) errI0 = errIMin;
        } else {
            if (errI0 < -errIMax) errI0 = -errIMax;
            if (errI0 > -errIMin) errI0 = -errIMin;
        }
        // The D parameter is equal to the acceleration
        errD = accel0;

        // Perform a weighted addition of errors to calculate power adjustment
        adjust = ((int32_t)errP * kP) >> 12;
        adjust += ((int32_t)errI0 * kI) >> 14;
        adjust -= ((int32_t)errD * kD) >> 12;
        //adjust += adjustX;

        // Since power is unsigned, and 'adjust' sees power as signed,
        // we must correct its sign
        if (targetSpeed0s < 0) adjust =-adjust;
        
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

    setPower800(0, power0);


    /* --- Motor1 CALCULATIONS: Identical to Motor0 --- */
    if (targetSpeed1s == 0) {
        power1 = 0;
    } else {
        errP = targetSpeed1s - speed1;
        errI1 += errP;
        if (targetSpeed1s > 0) {
            if (errI1 > errIMax) errI1 = errIMax;
            if (errI1 < errIMin) errI1 = errIMin;
        } else {
            if (errI1 < -errIMax) errI1 = -errIMax;
            if (errI1 > -errIMin) errI1 = -errIMin;
        }
        errD = accel1;

        adjust = ((int32_t)errP * kP) >> 12;
        adjust += ((int32_t)errI1 * kI) >> 14;
        adjust -= ((int32_t)errD * kD) >> 12;
        //adjust += adjustX;
        
        if (targetSpeed1s < 0) adjust =-adjust;
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

    setPower800(1, power1);
}

