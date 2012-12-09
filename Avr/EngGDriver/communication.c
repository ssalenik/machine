/** 
 * communication.c: 
 * Reads serial port and dispatches status & commands
 * DO NOT COMPILE, ONLY INCLUDE IN EngGDriver.c
 * For function prototypes and defines, see EngGDriver.h
 * For public variables, see variables.c
 */
 
 /**
  * Serial port dispatcher. Uses a buffered serial port.
  * See uart.c for the buffered serial port implementation.
  */
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
                    if (valid) setPower100(LMOTOR, arg1);
                    break;
                case 0x02: // set power of right motor
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) setPower100(RMOTOR, arg1);
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
                        setPower100(LMOTOR, arg1);
                        setPower100(RMOTOR, arg1);
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
                            targetSpeed1 = arg1 * 16;
                        } else {
                            targetSpeed0 = arg1 * 16;
                        }
                        resetPID();
                    }
                    break;
                case 0x12: // set desired speed of right motor
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        if (RMOTOR) {
                            targetSpeed1 = arg1 * 16;
                        } else {
                            targetSpeed0 = arg1 * 16;
                        }
                        resetPID();
                    }
                    break;
                case 0x15: // set speed for both motors
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        targetSpeed0 = arg1 * 16;
                        targetSpeed1 = arg1 * 16;
                        resetPID();
                    }
                    break;
                case 0x16: // reverse direction for both motors
                    ldir = (ldir == FORWARD) ? BACKWARD : FORWARD;
                    rdir = (rdir == FORWARD) ? BACKWARD : FORWARD;
                    setDirection(LMOTOR, ldir);
                    setDirection(RMOTOR, rdir);
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
                // NAVIGATOR COMMANDS
                /*case 0x30: // break, put navigator to idle
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
                    break;*/
                case 0x43: // toggle rotation angle adjustment
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) rotAdjOn = arg1;
                    break;
                //PID PARAMETERS
                case 0x60: // set kP, kI, kD, kX
                    arg1 = readByte(&buf[2], &valid);
                    arg2 = readByte(&buf[4], &valid);
                    arg3 = readByte(&buf[6], &valid);
                    if (valid) {
                        kP = arg1;
                        kI = arg2;
                        kD = arg3;
                    }
                    arg4 = readByte(&buf[8], &valid);
                    if (valid) {
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
                    debug1 = 0;
                    debug2 = 0;
                    debug3 = 0;
                    debug4 = 0;
                    debug5 = 0;
                    debug6 = 0;
                    debug7 = 0;
                    break;
                case 0x71:
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        debug1 = arg1;
                    } else {
                        debug1 = debug1 ? 0 : 1;
                    }
                    break;
                case 0x72:
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        debug2 = arg1;
                    } else {
                        debug2 = debug2 ? 0 : 1;
                    }
                    break;
                case 0x73:
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        debug3 = arg1;
                    } else {
                        debug3 = debug3 ? 0 : 1;
                    }
                    break;
                case 0x74:
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        debug4 = arg1;
                    } else {
                        debug4 = debug4 ? 0 : 1;
                    }
                    break;
                case 0x75:
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        debug5 = arg1;
                    } else {
                        debug5 = debug5 ? 0 : 1;
                    }
                    break;
                case 0x76:
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        debug6 = arg1;
                    } else {
                        debug6 = debug6 ? 0 : 1;
                    }
                    break;
                case 0x77:
                    arg1 = readByte(&buf[2], &valid);
                    if (valid) {
                        debug7 = arg1;
                    } else {
                        debug7 = debug7 ? 0 : 1;
                    }
                    break;
                case 0x78:
                    printParams();
                    break;
                case 0x79:
                    printParams2();
                    break;
                case 0x7f:
                    // set debug period
                    arg1i = readInt(&buf[2], &valid);
                    if (valid) {
                        if (arg1i && (uint16_t)arg1i < 100) arg1i = 100;
                        debugPeriod = (uint16_t)arg1i;
                    }
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

/** 
 * Convert 2 hex characters into a byte and return it.
 * If the ascii chars are not hex symbols, set valid to 0.
 * see hex.c for more info about parsing.
 */
inline uint8_t readByte(char *buf, uint8_t *valid) {
    *valid = *valid & isHex(buf[0]) & isHex(buf[1]);
    return htoi(buf[0], buf[1]);
}

/** 
 * Return 16-bit unsigned int. See readByte for more info.
 */
inline uint16_t readUInt(char *buf, uint8_t *valid) {
    *valid = *valid & isHex(buf[0]) & isHex(buf[1]) & 
        isHex(buf[2]) & isHex(buf[3]);
    return htoi(buf[0], buf[1]) << 8 | htoi(buf[2], buf[3]);
}

/**
 * Return 16-bit signed int. See readByte for more info.
 */
inline int16_t readInt(char *buf, uint8_t *valid) {
    *valid = *valid & isHex(buf[0]) & isHex(buf[1]) & 
        isHex(buf[2]) & isHex(buf[3]);
    return htoi(buf[0], buf[1]) << 8 | htoi(buf[2], buf[3]);
}

/* --- OBSOLETE functions below --- */

/** 
 * Sends done acknowledgement. 
 */
void sendDone() {
    uart_put('D', NULL);
    uart_put('\r', NULL);
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

// sends travelled distance in mm for each track
/*void sendDist() {
    int16_t ticks0_cached, ticks1_cached;
    int16_t dist0, dist1;
    cli();
    ticks0_cached = ticks0;
    ticks1_cached = ticks1;
    sei();
    dist0 = (int32_t) ticks0_cached * TICKSTODIST >> 8;
    dist1 = (int32_t) ticks1_cached * TICKSTODIST >> 8;
    printf("T%04x%04x\r", dist0, dist1);
}*/