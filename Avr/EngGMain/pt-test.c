static int pt_test(struct pt *pt) {
	PT_BEGIN(pt); // required to denote the beginning of a thread
	static uint32_t pt_target; // for delays
	
	// test thread 1
	if(run_test == 1) {
		;
	}
	
	// test thread 2
	else if(run_test == 2) {
		;
	}
	
	// test thread 3
	else if(run_test == 3) {
		;
	}
	
	run_test = 0; // stop test thread after execution
	//PT_WAIT_WHILE(pt, 1); // halt the thread to prevent restart
	PT_END(pt); // required to denote the beginning of a thread
}

