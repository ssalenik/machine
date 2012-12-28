static int pt_test(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	static uint32_t pt_target; // for delays
	
	// test thread 1: from beginning until shoot the target
	if(run_test == 1) {
		;
	}
	
	// test thread 2: from shoot the target to drop the battery
	else if(run_test == 2) {
		;
	}
	
	// test thread 3: from plank 34 across the turn, pick up egg and return at max speed
	else if(run_test == 3) {
		nav_forward(80, 80);
		PT_WAIT_UNTIL(pt, (abspL > rel2absL(37, 130)) || (abspR > rel2absR(37, 130)));
		pos_corr_off();
		nav_turnCCW(80, 80);
		SLEEP(1500);
		nav_forward(80, 80); PT_WAIT_UNTIL(pt, !trksR);
		set_rel_pos(40, 0);
		nav_rel_pos(80, 41,  20); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		pos_corr_on();
		nav_rel_pos(40, 40,  50); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		nav_rel_pos(80, 41, 115); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		claw_down(); claw_close(); SLEEP(800);
		claw_up(); SLEEP(SERVO_DELAY);
		nav_reverse(80, 80);
		PT_WAIT_UNTIL(pt, (abspL < rel2absL(40, -70)) || (abspR < rel2absR(40, -70)));
		pos_corr_off();
		nav_reverse(25, 80);
		SLEEP(1500);
		nav_reverse(80, 80); PT_WAIT_UNTIL(pt, !trksR);
		set_rel_pos(39, 0);
		PT_WAIT_UNTIL(pt, (abspL < rel2absL(38, 0)) || (abspR < rel2absR(38, 0)));
		pos_corr_on();
		nav_rel_pos(80, 35,  60); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		nav_rel_pos(80, 36,  70); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		laser_mid(); SLEEP(SERVO_DELAY);
		nav_rel_pos(80, 36,  33); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		laser_full(); SLEEP(3000);
		laser_mid(); SLEEP(SERVO_DELAY);
		nav_rel_pos(80, 36,  70); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		laser_init(); SLEEP(SERVO_DELAY);
		nav_rel_pos(DRIVE_SPEED, 1,   0); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop(); SLEEP(2000);
	}
	
	run_test = 0; // stop test thread after execution
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required to denote the beginning of a thread
}

