#define VBAT_LOW	767
#define VBAT_NORM	789

void check_sys_state(void) {
	set_interval(1000);
	
	uint16_t vbat = read_adc(VSENS);
	if(vbat < VBAT_LOW) set_bit(LED1);
	else if(vbat > VBAT_NORM) clr_bit(LED1);
}

void check_print_stat(void) {
	set_interval(500);
	
	if(local_dump) {
		//fprintf_P(&debug, PSTR("E%+d\tP%+d\tI%+ld\tD%+d\t\t"), read_enc3_synchro(), enc3_pro, enc3_int, enc3_der);
		//fprintf_P(&debug, PSTR("E%+d\tP%+d\tI%+ld\tD%+d\r\n"), read_actu_synchro(), actu_pro, actu_int, actu_der);
		
		fprintf_P(&debug, PSTR("<32%04x\r\n<33%04x\r\n<34%08lx\r\n<35%04x\r\n"), read_enc3_synchro(), enc3_pro, enc3_int, enc3_der);
		fprintf_P(&debug, PSTR("<42%04x\r\n<43%04x\r\n<44%08lx\r\n<45%04x\r\n"), read_actu_synchro(), actu_pro, actu_int, actu_der);
		
		fprintf_P(&debug, PSTR("R%+d\tR%+d\r\n"), read_enc3_ref(), read_actu_ref());
	}
}

