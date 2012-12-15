/** 
 * navigation.c: 
 * Navigation module that travels to requested positions.
 * DO NOT COMPILE, ONLY INCLUDE IN EngGDriver.c
 * For function prototypes and defines, see EngGDriver.h
 * For public variables, see variables.c
 *
 * NOTE: THIS MODULE IS OBSOLETE AND NEEDS REWORKING
 */
 
 /* function that decides where the robot goes by setting target speed of motors
 * Possible commands:
 *  - go straight continuously - NAV_SYNC
 *  - travel a given distance - NAV_DIST
 *  - free mode on each motor (includes free rotation) - NAV_FREE
 *  - rotate to angle - NAV_ROT;
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
    navCom = NAV_SYNC;
}

/*void navDist(int16_t speed, uint8_t dir, int16_t distance) {
    resetPID();
    if (dir != ldir) setDirection(LMOTOR, dir);
    if (dir != rdir) setDirection(RMOTOR, dir);
    n_targetHeading = heading;
    targetSpeed0 = speed;
    targetSpeed1 = speed;
    n_ticks = (int32_t) distance * DISTTOTICKS >> 8;
    navCom = NAV_DIST;
}*/

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

/** 
 *returns angle between endAng and startAng when rotating in a given direction
 * return value from -179 to 180
 * direction: ANG_CW or ANG_CCW
 */
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

 void setTargetSpeed(uint8_t motor, int16_t speed) {
    if (motor) {
        targetSpeed1 = speed;
    } else {
        targetSpeed0 = speed;
    }
}
