/** 
 * odometer.c: 
 * Odometer and Position Correction
 * DO NOT COMPILE, ONLY INCLUDE IN EngGDriver.c
 * For function prototypes and defines, see EngGDriver.h
 * For public variables, see variables.c
 *
 * The odometer has two types of positions: absolute and relative.
 * Both are in mm, however they use different starting points as
 * references. The absolute position starts at the very begining
 * of Section 1. The relative position starts from a transition 
 * point. There's a total of 48 transition points, 2 for each 
 * transversal track and thus 8 per section.
 * The track sensors also use those transition points as references
 * in order to do position correction.
 */

/** 
 * Set odometer absolute and relative positions to 0.
 */
void resetOdometer() {
    setOdometerTo(0, 0);
}

/**
 * Convert changes in tacho count to changes in position (in mm)
 * Also updates relative position by calling updateRelativePos().
 */
void runOdometer() {
    int32_t deltaLticks, deltaRticks;
    
    // read the change in tacho since last run
    cli();
    if (RMOTOR) {
        deltaLticks = ticks0 - p_Lticks;
        p_Lticks = ticks0;
        deltaRticks = ticks1 - p_Rticks;
        p_Rticks = ticks1;
    } else {
        deltaLticks = ticks1 - p_Lticks;
        p_Lticks = ticks1;
        deltaRticks = ticks0 - p_Rticks;
        p_Rticks = ticks0;
    }
    sei();
    
    // calculate change in distance and update absolute position
    p_Lfull += deltaLticks * TICKSTODIST_L;
    p_Rfull += deltaRticks * TICKSTODIST_R;
    p_L = p_Lfull >> 10;
    p_R = p_Rfull >> 10;
    
    // update relative position
    updateRelativePos();
}

/** 
 * Updates relative position using absolute positions p_L and p_R
 * Relative positions are using a list of 'transitions' as references.
 * Those are crossing points where transversal track planks start
 * or end. See p_transLlist and p_transRlist for crossing points.
 */
void updateRelativePos() {
    int16_t relToAbsL, relToAbsR;
    
    // update relative position
    relToAbsL = relativeToAbsolutePos_L(p_Lrel, p_Ltrans);
    p_Lrel = p_Lrel + p_L - relToAbsL;
    
    // if a there's a change in transition region, just run 
    // the absoluteToRelativePos function
    if (p_Lrel < 0) {
        p_Lrel = absoluteToRelativePos_L(p_L, &p_Ltrans);
    } else if (p_Ltrans < TRANSITIONS - 1) {
        if (p_Lrel + p_transLlist[p_Ltrans] >= p_transLlist[p_Ltrans + 1]) {
            p_Lrel = absoluteToRelativePos_L(p_L, &p_Ltrans);
        }
    }
    
    relToAbsR = relativeToAbsolutePos_R(p_Rrel, p_Rtrans);
    p_Rrel = p_Rrel + p_R - relToAbsR;
    
    if (p_Rrel < 0) {
        p_Rrel = absoluteToRelativePos_R(p_R, &p_Rtrans);
    } else if (p_Rtrans < TRANSITIONS - 1) {
        if (p_Rrel + p_transRlist[p_Rtrans] >= p_transRlist[p_Rtrans + 1]) {
            p_Rrel = absoluteToRelativePos_R(p_R, &p_Rtrans);
        }
    }
    
    /**
     * With the new position correction algorithm, position corrections are
     * only applied when a sensor passes from a gap to a plank, due to false
     * negative readings. Therefore, make sensors ready to detect planks only
     * in gaps (odd sections) and make them not ready when they pass to planks
     * (even sections). This eliminates multiple triggering and false negatives.
     */
    
    // make left sensor ready when necessary, depending on left motor direction
    /*if (p_Ltrans & 1) {
        if (ldir == FORWARD) {
            if (p_Lrel > p_transLlist[p_Ltrans+1] - p_transLlist[p_Ltrans] - MAX_CORR_ERROR) {
                posCorrLready = 1;
            }
        } else if (ldir == BACKWARD) {
            if (p_Lrel < MAX_CORR_ERROR) {
                posCorrLready = 1;
            }
        }
    }
    
    // make right sensor ready when necessary, depending on right motor direction
    if (p_Rtrans & 1) {
        if (rdir == FORWARD) {
            if (p_Rrel > p_transRlist[p_Rtrans+1] - p_transRlist[p_Rtrans] - MAX_CORR_ERROR) {
                posCorrRready = 1;
            }
        } else if (rdir == BACKWARD) {
            if (p_Rrel < MAX_CORR_ERROR) {
                posCorrRready = 1;
            }
        }
    }
    
    // make left sensor not ready when necessary, depending on left motor direction
    if (~p_Ltrans & 1) {
        if (ldir == FORWARD) {
            if (p_Lrel > MAX_CORR_ERROR) {
                posCorrLready = 0;
            }
        } else if (ldir == BACKWARD) {
            if (p_Lrel < p_transLlist[p_Ltrans+1] - p_transLlist[p_Ltrans] - MAX_CORR_ERROR) {
                posCorrLready = 0;
            }
        }
    }
    
    // make right sensor not ready when necessary, depending on left motor direction
    if (~p_Rtrans & 1) {
        if (rdir == FORWARD) {
            if (p_Rrel > MAX_CORR_ERROR) {
                posCorrRready = 0;
            }
        } else if (rdir == BACKWARD) {
            if (p_Rrel < p_transRlist[p_Rtrans+1] - p_transRlist[p_Rtrans] - MAX_CORR_ERROR) {
                posCorrRready = 0;
            }
        }
    }*/
}

/**
 * Set odometer to a given absolute position in mm.
 * Also updates relative position by calling updateRelativePos().
 */
void setOdometerTo(int16_t posL, int16_t posR) {
    // reset last odometery tacho to current tacho
    cli();
    if (RMOTOR) {
        p_Lticks = ticks0;
        p_Rticks = ticks1;
    } else {
        p_Lticks = ticks1;
        p_Rticks = ticks0;
    }
    sei();
    
    // update distances
    p_L = posL;
    p_R = posR;
    p_Lfull = ((int32_t)p_L) << 10;
    p_Rfull = ((int32_t)p_R) << 10;
    
    // update relative position
    updateRelativePos();
}

/**
 * Resets position correction.
 * Must be done before turning position correction on.
 */
void resetPosCorrection() {
    // update track sensor values
    readTrackSensors();
    // clear correction errors
    p_Lerr = p_Rerr = 0;
    // TODO: might cause false triggering if reset above a rail
    p_LlastCorrTrans = p_RlastCorrTrans = 255; 
}

/**
 * Reads track sensors and looks for Low-to-High and High-to-Low
 * transitions. Upon detection, it finds the closest transition
 * point from lookup tables and resets the odometer to it if the
 * error is within acceptable limits.
 */
void positionCorrection() {
    uint8_t lastLval, lastRval;
    int16_t transL, transR;
    int16_t absErr;
    uint8_t falseTrigger;
    
    // remember previous track sensor values
    lastLval = p_LsensVal;
    lastRval = p_RsensVal;
    
    // read the track sensors
    readTrackSensors();
    
    // exit if correction is turned off
    if (!posCorrectionOn) {
        return;
    }
    
    // check for transitions from hole to plank
    if ((lastLval && !p_LsensVal) || (lastRval && !p_RsensVal)) {
        /* make sure we have the updated position values if we need to
         * run the correction algorithm. Note that since the digital
         * sensor has a very fast smapling rate of about 2.5 ms, the 
         * speed of the robot has negligeable effect on the precision
         * of the correction.
         */
        runOdometer();
    }
    
    // check left side
    if (lastLval && !p_LsensVal) {
        // check if we are closer to the next transition than to the 
        // one in whose zone we are
        transL = p_Ltrans;
        if (p_Ltrans < TRANSITIONS - 1) {
            p_Lerr = (p_transLlist[p_Ltrans] + p_Lrel) - p_transLlist[p_Ltrans + 1];
            if (-p_Lerr < p_Lrel) {
                transL++;
            } else {
                p_Lerr = p_Lrel;
            }
        } else {
            p_Lerr = p_Lrel;
        }
        
        falseTrigger = 0;
        // TODO: add extra case for direction switching
        // check for false early triggering
        if ( (ldir == FORWARD) && (transL & 1) ) {
            falseTrigger = 1;
        } else if ( ( (transL & ~1) == (p_LlastCorrTrans & ~1) ) ) {
            falseTrigger = 1;
        // check for false double triggering
        } else if ((transL == p_LlastCorrTrans)) {
            falseTrigger = 1;
        }
        
        if (falseTrigger) {
            printf(" L false trigger\r\n"); // *** debug ***
        } else {
            // check that correction is whitin acceptable range
            absErr = p_Lerr < 0 ? -p_Lerr : p_Lerr;
            if (absErr <= MAX_CORR_ERROR) {
                // apply the correction
                p_Ltrans = p_LlastCorrTrans = transL;
                p_Lrel = 0;
                p_L = p_transLlist[transL];
                p_Lfull = ((int32_t)p_L) << 10;
                printf("L corr sect %u by %d at %ld\r\n", transL, p_Lerr, uptime());
            } else {
                // mucho problemo
                // TODO: send a notification
                posCorrLeftFailed++;
                printf("L corr failed!\r\n");
            }
        }
    }

    // check right side
    if (lastRval && !p_RsensVal) {
        transR = p_Rtrans;
        if (p_Rtrans < TRANSITIONS - 1) {
            p_Rerr = (p_transRlist[p_Rtrans] + p_Rrel) - p_transRlist[p_Rtrans + 1];
            if (-p_Rerr < p_Rrel) {
                transR++;
            } else {
                p_Rerr = p_Rrel;
            }
        } else {
            p_Rerr = p_Rrel;
        }

        falseTrigger = 0;
        // TODO: add extra case for direction switching
        // check for false early triggering
        if ( (rdir == FORWARD) && (transR & 1) ) {
            falseTrigger = 1;
        } else if ( (rdir == BACKWARD) && !(transR & 1) ) {
            falseTrigger = 1;
        // check for false double triggering
        } else if ( (transR & ~1) == (p_RlastCorrTrans & ~1) ) {
            falseTrigger = 1;
        }
        
        if (falseTrigger) {
            //printf(" R false trigger\r\n"); // *** debug ***
        } else {
            absErr = p_Rerr < 0 ? -p_Rerr : p_Rerr;
            if (absErr <= MAX_CORR_ERROR) {
                p_Rtrans = p_RlastCorrTrans = transR;
                p_Rrel = 0;
                p_R = p_transRlist[transR];
                p_Rfull = ((int32_t)p_R) << 10;
                printf("R corr sect %u by %d at %ld\r\n", transR, p_Rerr, uptime());
            } else {
                // mucho problemo
                // TODO: send a notification
                posCorrRightFailed++;
                printf("R corr failed!\r\n");
            }
        }
    }    
}

/**
 * Read track sensors.
 */
// digital version
void readTrackSensors() {
    p_LsensVal = PINC & 0x01;
    p_RsensVal = (PINC >> 1) & 0x01;
}
/*
// analog version
void readTrackSensors() {
    cli();
    uint16_t tmpL = p_LsensADC;
    uint16_t tmpR = p_RsensADC;
    sei();
    p_LsensVal = tmpL > POS_SENSOR_THR ? 0 : 1;
    p_RsensVal = tmpR > POS_SENSOR_THR ? 0 : 1;
}
*/

/**
 * Convert absolute position to relative position for left motor
 * by using lookup table p_transLlist (all in mm).
 */
int16_t absoluteToRelativePos_L(int16_t absPosL, int8_t *pTransition) {
    uint8_t trans = 0;
   
    // split the search list in 4 for efficiency
    if (absPosL >= p_transLlist[TRANSITIONS / 2]) {
        if (absPosL >= p_transLlist[(3 * TRANSITIONS) / 4]) {
            trans = (3 * TRANSITIONS) / 4;
            // special case, location >= Last Transition 
            if (absPosL >= p_transLlist[TRANSITIONS - 1] ) {
                *pTransition = TRANSITIONS - 1;
                return absPosL - p_transLlist[TRANSITIONS - 1];
            }
        } else {
            trans = TRANSITIONS / 2;
        }
    } else {
        if (absPosL >= p_transLlist[TRANSITIONS / 4]) {
            trans = TRANSITIONS / 4;
        } else {
            trans = 0;
        }
    }
    
    while(++trans < TRANSITIONS) {
        if (absPosL < p_transLlist[trans]) break;
    }
    
    trans--;
    *pTransition = trans;
    return absPosL - p_transLlist[trans];
}

/**
 * Convert absolute position to relative position for right motor
 * by using lookup table p_transRlist (all in mm).
 */
int16_t absoluteToRelativePos_R(int16_t absPosR, int8_t *pTransition) {
    uint8_t trans = 0;
   
    // split the search list in 4 for efficiency
    if (absPosR >= p_transRlist[TRANSITIONS / 2]) {
        if (absPosR >= p_transRlist[(3 * TRANSITIONS) / 4]) {
            trans = (3 * TRANSITIONS) / 4;
            // special case, location >= Rast Transition 
            if (absPosR >= p_transRlist[TRANSITIONS - 1]) {
                *pTransition = TRANSITIONS - 1;
                return absPosR - p_transRlist[TRANSITIONS - 1];
            }
        } else {
            trans = TRANSITIONS / 2;
        }
    } else {
        if (absPosR >= p_transRlist[TRANSITIONS / 4]) {
            trans = TRANSITIONS / 4;
        } else {
            trans = 0;
        }
    }
    
    while(++trans < TRANSITIONS) {
        if (absPosR < p_transRlist[trans]) break;
    }
    
    trans--;
    *pTransition = trans;
    return absPosR - p_transRlist[trans];
}

/**
 * Convert relative position to absolute position for left motor.
 */
int16_t relativeToAbsolutePos_L(int16_t relPosL, uint8_t transition) {
    return(relPosL + p_transLlist[transition]);
}

/**
 * Convert relative position to absolute position for right motor.
 */
int16_t relativeToAbsolutePos_R(int16_t relPosR, uint8_t transition) {
    return(relPosR + p_transRlist[transition]);
}
