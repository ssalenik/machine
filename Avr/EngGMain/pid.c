/*
#define ENC3_P 10
#define ENC3_I  0
#define ENC3_D  0

#define ACTU_P  4
#define ACTU_I  0
#define ACTU_D  0
*/

int16_t ENC3_P = 10, ENC3_I = 0, ENC3_D = 0, ENC3_NOISE_GATE = 20, ACTU_P = 4, ACTU_I = 0, ACTU_D = 0, ACTU_NOISE_GATE = 15;

#define ENC3_P_SH 0
#define ENC3_I_SH 0
#define ENC3_D_SH 0

#define ACTU_P_SH 0
#define ACTU_I_SH 0
#define ACTU_D_SH 0

/*
#define ENC3_NOISE_GATE 20
#define ACTU_NOISE_GATE 10
*/

#define ENC3_INT_CAP 10000
#define ACTU_INT_CAP 10000

#define ENC3_PCAP 3000
#define ACTU_PCAP 3000

int16_t enc3_error_last, actu_error_last;
int16_t enc3_pro, actu_pro, enc3_der, actu_der;
int32_t enc3_int = 0, actu_int = 0;
uint8_t pid_on = 0;

void reset_pid(void) {
	write_enc3_ref(read_enc());
	write_actu_ref(read_adc(LIFT_FB));
	enc3_error_last = actu_error_last = enc3_int = actu_int = 0;
}

void update_pid_vals(void) {
	if(!run_pid || !pid_on) return;
	run_pid = 0;
	
	int16_t enc3_pro = read_enc3_error();
	int16_t actu_pro = read_actu_error();
	
	// PID for motor 3 (turn)
	if(abs(enc3_pro) < ENC3_NOISE_GATE) enc3_pro = enc3_int = 0;
	enc3_int += enc3_pro;
	if(labs(enc3_int) > ENC3_INT_CAP) {
		if(enc3_int < 0) enc3_int = -ENC3_INT_CAP;
		else             enc3_int =  ENC3_INT_CAP;
	}
	int32_t enc3_out = ((int32_t)enc3_pro * ENC3_P) >> ENC3_P_SH;
	enc3_out += (enc3_int * ENC3_I) >> ENC3_I_SH;
	enc3_der = enc3_pro - enc3_error_last;
	//enc3_out += ((int32_t)((enc3 - enc3_last) - (enc3_ref - enc3_ref_last)) * ENC3_D) >> ENC3_D_SH;
	enc3_error_last = enc3_pro;
	
	// PID for motor 4 (lift)
	if(abs(actu_pro) < ACTU_NOISE_GATE) actu_pro = actu_int = 0;
	actu_int += actu_pro;
	if(labs(actu_int) > ACTU_INT_CAP) {
		if(actu_int < 0) actu_int = -ACTU_INT_CAP;
		else             actu_int =  ACTU_INT_CAP;
	}
	int32_t actu_out = ((int32_t)actu_pro * ACTU_P) >> ACTU_P_SH;
	actu_out += (actu_int * ACTU_I) >> ACTU_I_SH;
	enc3_der = actu_pro - actu_error_last;
	//actu_out += ((int32_t)((actu - actu_last) - (actu_ref - actu_ref_last)) * ACTU_D) >> ACTU_D_SH;
	actu_error_last = actu_pro;
	
	// control of motor 3 (turn)
	if(enc3_out < 0) motor3_rev();
	else             motor3_fwd();
	enc3_out = labs(enc3_out);
	if(enc3_out > ENC3_PCAP) enc3_out = ENC3_PCAP;
	set_speed_3((int16_t)enc3_out);
	
	// control of motor 4 (lift)
	if(actu_out < 0) motor4_rev();
	else             motor4_fwd();
	actu_out = labs(actu_out);
	if(actu_out > ACTU_PCAP) actu_out = ACTU_PCAP;
	set_speed_4((int16_t)actu_out);
}

