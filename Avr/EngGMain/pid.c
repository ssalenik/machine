#define ENC3_P 10
#define ENC3_I  0
#define ENC3_D  0

#define ENC3_P_SH 0
#define ENC3_I_SH 0
#define ENC3_D_SH 0

#define ACTU_P  4
#define ACTU_I  0
#define ACTU_D  0

#define ACTU_P_SH 0
#define ACTU_I_SH 0
#define ACTU_D_SH 0

#define ENC3_NOISE_GATE 20
#define ACTU_NOISE_GATE 10

#define ENC3_INT_CAP 10000
#define ACTU_INT_CAP 10000

#define ENC3_PCAP 2000
#define ACTU_PCAP 2000

int16_t enc3_last, actu_last, enc3_ref_last, actu_ref_last;
int16_t enc3_pro, actu_pro, enc3_der, actu_der;
int32_t enc3_int = 0, actu_int = 0;
uint8_t pid_on = 0;

void reset_pid(void) {
	actu_last = actu_ref_last = read_adc(LIFT_FB);
	write_actu_ref(actu_last);
	enc3_last = enc3_ref_last = read_enc();
	write_enc3_ref(enc3_last);
}

void update_pid_vals(void) {
	if(!run_pid || !pid_on) return;
	run_pid = 0;
	
	int16_t enc3 = read_enc3_synchro();
	int16_t actu = read_actu_synchro();
	int16_t enc3_ref = read_enc3_ref();
	int16_t actu_ref = read_actu_ref();
	
	enc3_pro = enc3 - enc3_ref;
	if(abs(enc3_pro) < ENC3_NOISE_GATE) enc3_pro = 0;
	enc3_int += enc3_pro;
	if(abs(enc3_int) > ENC3_INT_CAP) {
		if(enc3_int < 0) enc3_int = -ENC3_INT_CAP;
		else             enc3_int =  ENC3_INT_CAP;
	}
	int32_t enc3_out = ((int32_t)enc3_pro * ENC3_P) >> ENC3_P_SH;
	//enc3_out += (enc3_int * ENC3_I) >> ENC3_I_SH;
	enc3_der = (enc3 - enc3_last) - (enc3_ref - enc3_ref_last);
	//enc3_out += ((int32_t)((enc3 - enc3_last) - (enc3_ref - enc3_ref_last)) * ENC3_D) >> ENC3_D_SH;
	
	actu_pro = actu - actu_ref;
	if(abs(actu_pro) < ACTU_NOISE_GATE) actu_pro = 0;
	actu_int += actu_pro;
	if(abs(actu_int) > ACTU_INT_CAP) {
		if(actu_int < 0) actu_int = -ACTU_INT_CAP;
		else             actu_int =  ACTU_INT_CAP;
	}
	int32_t actu_out = ((int32_t)actu_pro * ACTU_P) >> ACTU_P_SH;
	//actu_out += (actu_int * ACTU_I) >> ACTU_I_SH;
	actu_der = (actu - actu_last) - (actu_ref - actu_ref_last);
	//actu_out += ((int32_t)((actu - actu_last) - (actu_ref - actu_ref_last)) * ACTU_D) >> ACTU_D_SH;
	
	if(enc3_out < 0) motor3_fwd();
	else             motor3_rev();
	enc3_out = abs(enc3_out);
	if(enc3_out > ENC3_PCAP) enc3_out = ENC3_PCAP;
	set_speed_3((int16_t)enc3_out);
	
	if(actu_out < 0) motor4_fwd();
	else             motor4_rev();
	actu_out = abs(actu_out);
	if(actu_out > ACTU_PCAP) actu_out = ACTU_PCAP;
	set_speed_4((int16_t)actu_out);
	
	enc3_last = enc3;
	actu_last = actu;
	enc3_ref_last = enc3_ref;
	actu_ref_last = actu_ref;
}

