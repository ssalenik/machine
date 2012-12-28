static int pt_test(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	static uint32_t pt_target; // for delays
	
	// test thread 1: from plank 34 across the turn, pick up egg and return at max speed
	if(run_test == 1) {
		set_rel_pos(34, 50); // FOR TEST ONLY
		pos_corr_on(); // FOR TEST ONLY
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
	
	// test thread 2: from beginning to lifting the barrier
	else if(run_test == 2) {
		nav_actu(3, 400); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
		nav_base(20, 180); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
		set_rel_pos(0, 30);
		pos_corr_on();
		nav_rel_pos(DRIVE_SPEED, 17,  70);
		PT_WAIT_UNTIL(pt, (abspL > rel2absL(9, 60)) || (abspR > rel2absR(9, 60)));
		nav_base(20, 90);
		nav_actu(1, 350);
		PT_WAIT_UNTIL(pt, (abspL > rel2absL(15, 0)) || (abspR > rel2absR(15, 0)));
		// tower gets hit here
		nav_base(20, -45);
		PT_WAIT_UNTIL(pt, bit_clr(Q3_Z)); // turn zero signal
		cli();
		V_encoder = (5 * ARM_TURN_FACTOR) >> 3; // 5 deg.
		sei();
		set_speed_3(0);
		set_speed_4(0);
		pid_on = 0;
		SLEEP(600);
		reset_pid();
		pid_on = 1;
		PT_WAIT_UNTIL(pt, drive_complete);
		nav_base(10, -5);
		nav_actu(2, 40);
		PT_WAIT_UNTIL(pt, pid_complete[MOTOR3] && pid_complete[MOTOR4]);
		nav_rel_pos(DRIVE_SPEED, 18,  60); PT_WAIT_UNTIL(pt, drive_complete);
	}
	
	// test thread 3: lift the barrier, grab the battry and shoot the target
	else if(run_test == 3) {
		set_rel_pos(20, 75); // FOR TEST ONLY
		pos_corr_on(); // FOR TEST ONLY
		nav_actu(4, 600); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
		// multi-step speed profile (acceleration)
		nav_base(20,  -5); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
		nav_base(40, -15); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
		nav_base(60, -30); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
		nav_base(80, -75); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
		// multi-step speed profile (deceleration)
		nav_base(60, -105); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
		nav_base(40, -120); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
		nav_base(20, -125); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
		// end of fast turn
		nav_rel_pos(DRIVE_SPEED, 11,  20);
		nav_base(20, -85);
		nav_actu(3, 200);
		PT_WAIT_UNTIL(pt, drive_complete && pid_complete[MOTOR3] && pid_complete[MOTOR4]);
		SLEEP(10000); // TEST
		nav_actu(1, 100); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
		SLEEP(10000); // TEST
		set_bit(FET1); set_bit(FET2);
		nav_actu(1, 200); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
		nav_actu(1, 350); // TARGET HEIGHT
		nav_rel_pos(DRIVE_SPEED, 27, 100); // TARGET POSITION
		PT_WAIT_UNTIL(pt, drive_complete && pid_complete[MOTOR4]);
		cannon_shoot();
	}
	
	run_test = 0; // stop test thread after execution
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required to denote the beginning of a thread
}

