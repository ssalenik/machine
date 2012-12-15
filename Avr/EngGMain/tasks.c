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
		fprintf_P(&debug, PSTR("E%+d\tR%+d\tP%+d\tI%+ld\tD%+d\t\t"), read_enc(),        pid_ref[MOTOR3], enc3_pro, enc3_int, enc3_der);
		fprintf_P(&debug, PSTR("E%+d\tR%+d\tP%+d\tI%+ld\tD%+d\r\n"), read_adc(LIFT_FB), pid_ref[MOTOR4], actu_pro, actu_int, actu_der);
		
		//fprintf_P(&debug, PSTR("<32%04x\r\n<33%04x\r\n<34%08lx\r\n<35%04x\r\n"), read_enc(),        enc3_pro, enc3_int, enc3_der);
		//fprintf_P(&debug, PSTR("<42%04x\r\n<43%04x\r\n<44%08lx\r\n<45%04x\r\n"), read_adc(LIFT_FB), actu_pro, actu_int, actu_der);
		
		//fprintf_P(&debug, PSTR("R%+d\tR%+d\r\n"), V_enc3_ref, V_actu_ref);
	}
}
