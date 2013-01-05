static int pt_main(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	static uint32_t pt_target; // for delays
	//static uint32_t sc_start; // for short-circuit delay
	static uint8_t i; // for loops
	
	/* --- COUNTDOWN --- */
	fprintf(&debug, "RUNNING!\r\n"); // so u know when to disable the thread for tests
	//SLEEP(5000); // this delay allows GTF away from the robot, or to stop the main thread before run
	fprintf(&debug, "5\r\n"); SLEEP(1000);
	fprintf(&debug, "4\r\n"); SLEEP(1000);
	fprintf(&debug, "3\r\n"); SLEEP(1000);
	fprintf(&debug, "2\r\n"); SLEEP(1000);
	fprintf(&debug, "1\r\n"); SLEEP(1000);
	
	/* --- PASS THE BRIDGE --- */
	// robot must be placed 30 mm from transition 0, i.e. 20 mm forward from start of tracks
	// with the actuator pointing left 90 degrees, raised to position 400 ideally
	// pos = 0.30
	nav_actu(3, 400); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	nav_base(20, 180); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
	set_rel_pos(0, 30);
	pos_corr_on();
	nav_rel_pos(DRIVE_SPEED, 17,  60); // start moving under the bridge, until barrier
	PT_WAIT_UNTIL(pt, (abspL > rel2absL(9, 60)) || (abspR > rel2absR(9, 60)));
	// passed the bridge
	nav_base(20, 90);
	nav_actu(1, 350);
	PT_WAIT_UNTIL(pt, (abspL > rel2absL(15, 0)) || (abspR > rel2absR(15, 0)));
	// tower gets hit here
	
	/* --- RESET BASE ENCODER --- */
	// pos = ?, probably 17.60. base = 90L
	// turn the base CW until the abs reference point is crossed (stops at 45 deg Right on failure)
	nav_base(20, -45);
	PT_WAIT_UNTIL(pt, bit_clr(Q3_Z)); // detect base ref point
	cli();
	// reset base tacho. reference point is at 5 deg Left
	V_encoder = deg2ticks(5);
	sei();
	// clear all pid variables and sleep to let the pid settle
	set_speed_3(0);
	set_speed_4(0);
	pid_on = 0;
	SLEEP(600);
	reset_pid();
	pid_on = 1;
	// got precise base reference here
	PT_WAIT_UNTIL(pt, drive_complete); // barrier reached
	
	/* --- LIFT BARRIER --- */
	// pos = 17.60. positioned in front of the barrier here. 
	nav_base(10, -5);
	nav_actu(2, 40);
	PT_WAIT_UNTIL(pt, pid_complete[MOTOR3] && pid_complete[MOTOR4]);
	nav_rel_pos(DRIVE_SPEED, 18,  80); PT_WAIT_UNTIL(pt, drive_complete);
	// positioned with arm under the barrier
	
	// Lift arm fully
	nav_actu(4, 400); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
	nav_rel_pos(DRIVE_SPEED, 18,  130);
	PT_WAIT_UNTIL(pt, pid_complete[MOTOR4] && drive_complete);
	nav_actu(4, 600);
	nav_rel_pos(DRIVE_SPEED, 18,  150);
	PT_WAIT_UNTIL(pt, pid_complete[MOTOR4] && drive_complete);
	nav_actu(4, 750); 
	nav_rel_pos(50, 18,  180);
	PT_WAIT_UNTIL(pt, pid_complete[MOTOR4] && drive_complete);
	
	// rotate arm
	nav_base( 25,  -30); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);		
	nav_base( 10,  -90); 
	nav_rel_pos(50, 21,  140);
	PT_WAIT_UNTIL(pt, (abspL > rel2absL(18, 250)) );
	nav_rel_pos(100, 21,  140);	
	PT_WAIT_UNTIL(pt, ref_complete[MOTOR3] && drive_complete);
	
	// lower arm
	nav_actu(4, 300); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
	
	// move out of the way of the barrier
	nav_actu(4, 400);
	nav_base( 40,  -50); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]); // rotate a bit not to crash into barrier
	nav_rel_pos(DRIVE_SPEED, 11,  20); // drive to battery
	PT_WAIT_UNTIL(pt, (abspL > rel2absL(18, 0)) );
	nav_base(10, -82); // rotate base to battery pick-up pos
	PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
	nav_actu(3, 200);
	PT_WAIT_UNTIL(pt, ref_complete[MOTOR4] && drive_complete); // wait until destination reached
	
	/* --- PICK UP BATTERY --- */
	// pos = 11.20, base = 82R, actu = 200 after barrier lifting.
	// actuator ready to pick up the battery, magnets on
	set_bit(FET1); set_bit(FET2);
	// make 3 attempts at different positions
	nav_actu(2,  75); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	nav_actu(2, 120); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	nav_dist(30, -20); PT_WAIT_UNTIL(pt, drive_complete);
	nav_actu(2,  75); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	nav_actu(2, 120); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	nav_dist(30, -20); PT_WAIT_UNTIL(pt, drive_complete);
	nav_actu(2,  75); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	nav_actu(2, 120); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
	// finished 3 attempts to pick up the battery
	
	/* --- SHOOT TARGET --- */
	// pos = 11.-20, base = 85R, actu = 120 after battery pickup.
	nav_base(10, -30); // required to bypass the barrier safely
	nav_actu(2, 200); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
	// start moving smoothly once actuator high enough
	nav_actu(3, 600); SLEEP(300); // TARGET HEIGHT
	nav_forward( 40,  40); SLEEP(300); // smooth acceletation
	nav_forward( 80,  80); SLEEP(300);
	nav_forward(120, 120); SLEEP(300);
	nav_forward(160, 160); SLEEP(300);
	nav_rel_pos(200, 27, 100); // TARGET POSITION
	PT_WAIT_UNTIL(pt, (abspL > rel2absL(21, 0)) || (abspR > rel2absR(21, 0)));
	// aim for target
	nav_base(10, -82);
	PT_WAIT_UNTIL(pt, drive_complete && pid_complete[MOTOR3] && pid_complete[MOTOR4]);
	SLEEP(400); // shoot once stabilized
	// shoot (try 3 times fast)
	for(i = 0; i < 3; i++) {
		cannon_shoot(); SLEEP(250);
		cannon_reload(); SLEEP(250);
	}
	
	/* --- BATTERY DROP-OFF & GEN SHORT CIRCUIT --- */
	// pos = 27.100, base = 82R, actu = 575 after target shot.
	// rotate the base to center without descent
	nav_base(10, 0); SLEEP(300);
	// go smoothly to short-circuit metal plates
	nav_forward( 40,  40); SLEEP(300); // smooth acceletation
	nav_forward( 80,  80); SLEEP(300);
	nav_forward(120, 120); SLEEP(300);
	nav_forward(160, 160); SLEEP(300);
	nav_rel_pos(200, 35,  0); SLEEP(300);
	// complete rotation and descend the actuator
	nav_base(20, 0); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
	nav_actu(3, 250);
	nav_base(20, 45); PT_WAIT_UNTIL(pt, ref_complete[MOTOR3]);
	nav_base(10, 65);
	PT_WAIT_UNTIL(pt, drive_complete);
	// stable short-circuit begins here
	//sc_start = uptime();
	//PT_WAIT_UNTIL(pt, uptime() > sc_start + 1500); // make sure short-circuit is at least 1.5 s
	SLEEP(1500);
	nav_forward(40, 40);
	PT_WAIT_UNTIL(pt, (abspL > rel2absL(35, 50)) || (abspR > rel2absR(35, 50)));
	// drop the battery
	clr_bit(FET1); clr_bit(FET2);
	
	/* --- TURN --- */
	// pos = 35.50, speed = 40, base = 75L, actu = 100 after battery drop-off / short.
	// start moving forward and position arm behind for traction
	nav_forward(80, 80);
	//nav_actu(3, 250); PT_WAIT_UNTIL(pt, ref_complete[MOTOR4]);
	// actu is at a safe height to proceed moving now
	nav_actu(3, 400); // lift actu to cruise height
	nav_base(20, 180);
	PT_WAIT_UNTIL(pt, (abspL > rel2absL(37, 130)) || (abspR > rel2absR(37, 130)));
	// turn off pos correction and initiate turn.
	pos_corr_off();
	nav_turnCCW(80, 80);
	SLEEP(1500); // travel a bit
	// wait until next section detected with Right track sensor
	nav_forward(80, 80); PT_WAIT_UNTIL(pt, !trksR);
	set_rel_pos(40, 0); // reset position
	// drive until a straight section
	nav_rel_pos(80, 41,  20); PT_WAIT_UNTIL(pt, drive_complete);
	// turn correction on and force it on trans 41 by driving backward
	pos_corr_on();
	nav_rel_pos(40, 40,  50); PT_WAIT_UNTIL(pt, drive_complete);
	
	/* --- PICK-UP EGG --- */
	// pos = 40.50, base = 180L, actu = 400 after turn.
	nav_rel_pos(80, 41, 115); PT_WAIT_UNTIL(pt, drive_complete);
	claw_down(); claw_close(); SLEEP(800);
	claw_up(); SLEEP(SERVO_DELAY);
	// egg picked-up
	
	/* --- REVERSE TURN --- */
	// pos = 40.50, base = 180L, actu = 400 after egg pick-up.
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
	
	/* --- LASER --- */
	// pos = 35.60, base = 180L, actu = 400 after reverse turn.
	// flag down
	nav_rel_pos(80, 36,  70); PT_WAIT_UNTIL(pt, drive_complete);
	laser_mid(); SLEEP(SERVO_DELAY);
	nav_rel_pos(80, 36,  33); PT_WAIT_UNTIL(pt, drive_complete);
	laser_full(); SLEEP(3000); // block laser here
	// flag up
	laser_mid(); SLEEP(SERVO_DELAY);
	nav_rel_pos(80, 36,  70); PT_WAIT_UNTIL(pt, drive_complete);
	laser_init(); SLEEP(SERVO_DELAY);
	
	/* --- BACK HOME BABY --- */
	// pos = 36.70, base = 180L, actu = 400 after laser.
	nav_rel_pos(DRIVE_SPEED, 2,   0);
	PT_WAIT_UNTIL(pt, (abspL < rel2absL(18, 0)) || (abspR < rel2absR(18, 0)));
	nav_actu(3, 300); // actuator down to a safe height after the slope
	PT_WAIT_UNTIL(pt, drive_complete);
	nav_actu(2, 50); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	
	/* --- TEST: RESET THE ARM POSITION --- */
	SLEEP(5000);
	nav_actu(3, 400); PT_WAIT_UNTIL(pt, pid_complete[MOTOR4]);
	nav_base(10, 90); PT_WAIT_UNTIL(pt, pid_complete[MOTOR3]);
	nav_rel_pos(DRIVE_SPEED, 0,  30); PT_WAIT_UNTIL(pt, drive_complete);
	
	//stop EVERYTHING. END OF EXECUTION
	run_main = 0;
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required as a denote the end of a thread
}


