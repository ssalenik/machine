#define VBAT_LOW	767
#define VBAT_NORM	789

void check_batt_voltage(void) {
	set_interval(1000);
	
	uint16_t vbat = read_adc(VSENS);
	if(vbat < VBAT_LOW) set_bit(LED1);
	else if(vbat > VBAT_NORM) clr_bit(LED1);
}

