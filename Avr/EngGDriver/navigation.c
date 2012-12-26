/** 
 * navigation.c: 
 * Navigation module that travels to requested positions.
 * DO NOT COMPILE, ONLY INCLUDE IN EngGDriver.c
 * For function prototypes and defines, see EngGDriver.h
 * For public variables, see variables.c
 *
 * NOTE: THIS MODULE NEEDS CLEANING
 */

/** 
 * function that decides where the robot goes by setting target speed of motors
 * Possible commands:
 *  - travel to a given absolute position - NAV_DIST
 *  - free mode on each motor (includes free rotation) - NAV_FREE
 */
void navigator() {
    int16_t distLeft;
    
    switch(navCom) {
    case NAV_NONE: break; // idle state
    case NAV_DIST:
        if (!n_Ldone) {
            distLeft = getDistLeft(ldir, n_targetLpos, p_L);
            // TODO: do not assume motor0 is motorL
            targetSpeed0 = calculateTargetSpeed(distLeft, targetSpeed0);
            if (targetSpeed0 == 0) n_Ldone = 1;
            if (debug6) printf_P(PSTR("%d\r\n"), targetSpeed0);
            if (debug5) printf_P(PSTR("%d\t%d\r\n"), targetSpeed0, speed0);
        }
        if (!n_Rdone) {
            distLeft = getDistLeft(rdir, n_targetRpos, p_R);
            // TODO: do not assume motor0 is motorL
            targetSpeed1 = calculateTargetSpeed(distLeft, targetSpeed1);
            if (targetSpeed1 == 0) n_Rdone = 1;
        }
        if (n_Ldone && n_Rdone) navCom = NAV_NONE;
        break;
    case NAV_FREE:
        // do nothing
        break;
    }
}

void navDestRel(int16_t speed, uint8_t transL, int16_t offsetL, 
    uint8_t transR, int16_t offsetR) 
{
    if (transL >= TRANSITIONS || transR >= TRANSITIONS) return;
    navDest(speed, p_transLlist[transL] + offsetL, p_transRlist[transR] + offsetR);
}

void navDest(int16_t speed, int16_t posL, int16_t posR) {
    targetSpeed0 = speed;
    targetSpeed1 = speed;
    n_targetLpos = posL;
    n_targetRpos = posR;
    n_Ldone = 0;
    n_Rdone = 0;
    ldir = (p_L < posL) ? FORWARD : BACKWARD;
    rdir = (p_R < posR) ? FORWARD : BACKWARD;
    setDirection(LMOTOR, ldir);
    setDirection(RMOTOR, rdir);
    printf_P(PSTR("Left: goto %d at speed %d, dir = %d\r\n"), posL, speed, ldir);
    printf_P(PSTR("Right: goto %d at speed %d, dir = %d\r\n"), posR, speed, rdir);
    navCom = NAV_DIST;
}

void navFree(int16_t speedL, int16_t speedR, uint8_t dirL, uint8_t dirR) {
    resetPID();
    if (dirL != ldir) setDirection(LMOTOR, dirL);
    if (dirR != rdir) setDirection(RMOTOR, dirR);
    setTargetSpeed(LMOTOR, speedL);
    setTargetSpeed(RMOTOR, speedR);
    if (speedL || speedR) {
        navCom = NAV_FREE;
    } else {
        navCom = NAV_NONE; // if speeds are passed as 0 then stop and put navigator in idle mode
    }
}

 void setTargetSpeed(uint8_t motor, int16_t speed) {
    if (motor) {
        targetSpeed1 = speed;
    } else {
        targetSpeed0 = speed;
    }
}

int16_t getDistLeft(uint8_t dir, int16_t target, int16_t current) {
    if (dir == FORWARD) {
        return target - current;
    } else {
        return current - target;
    }
}

/**
 * Calculate the new target speed based on how close to destination
 * all speeds are positive
 */
int16_t calculateTargetSpeed(int16_t distLeft, int16_t curTargetSpeed) {
    int16_t rampSpeed;
    if (distLeft <= RAMPDOWN_STOP_DIST) return 0;
    if (distLeft <= RAMPDOWN_CHECK_DIST) {
        rampSpeed = RAMPDOWN_RATE * (distLeft - RAMPDOWN_REF_DIST);
        rampSpeed = (rampSpeed < curTargetSpeed) ? rampSpeed : curTargetSpeed;
        if (rampSpeed < RAMPDOWN_MIN_SPEED) rampSpeed = RAMPDOWN_MIN_SPEED;
        return rampSpeed;
    }
    return curTargetSpeed;
}
