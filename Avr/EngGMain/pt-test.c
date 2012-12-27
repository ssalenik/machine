static int pt_test(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	static uint32_t pt_target; // for delays
	
	// test thread 1
	if(run_test == 1) {
		set_rel_pos(34, 50);
		pos_corr_on();
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
		servo7(160);
		servo8(160);
		SLEEP(800);
		servo7(96);
		SLEEP(400);
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
		servo6(58); SLEEP(400);
		nav_rel_pos(80, 36,  33); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		servo6(96); SLEEP(3000);
		servo6(58); SLEEP(400);
		nav_rel_pos(80, 36,  70); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop();
		servo6(4); SLEEP(400);
		nav_rel_pos(200, 1,   0); PT_WAIT_UNTIL(pt, drive_complete);
		nav_stop(); SLEEP(2000);
	}
	
	// test thread 2
	else if(run_test == 2) {
	}
	
	run_test = 0; // stop test thread after execution
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required to denote the beginning of a thread
}

