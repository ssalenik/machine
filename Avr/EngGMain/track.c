int16_t p_transLlist[TRANSITIONS] = TRANS_L_LIST;
int16_t p_transRlist[TRANSITIONS] = TRANS_R_LIST;

/**
 * Convert relative position to absolute position for left motor.
 */
int16_t rel2absL(uint8_t sectL, int16_t relpL) {
	return p_transLlist[sectL] + relpL;
}

/**
 * Convert relative position to absolute position for right motor.
 */
int16_t rel2absR(uint8_t sectR, int16_t relpR) {
	return p_transRlist[sectR] + relpR;
}

