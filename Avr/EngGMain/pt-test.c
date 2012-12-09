static int pt_test(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	static uint32_t pt_target; // for delays
	
	if(run_test == 1) {
		freedrive(0, 0, 0); sleep(100);
		stop(); sleep(100);
	}
	else if(run_test == 2) {
		freedrive(0, 0, 0); sleep(100);
		stop(); sleep(100);
	}
	
	run_test = 0; // stop test thread after execution
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required to denote the beginning of a thread
}

