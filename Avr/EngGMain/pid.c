#define ENC3_P  20
#define ENC3_I   3
#define ENC3_D 160

#define ACTU_P  96
#define ACTU_I   5
#define ACTU_D 160

#define ENC3_P_SH 0
#define ENC3_I_SH 0
#define ENC3_D_SH 0

#define ACTU_P_SH 0
#define ACTU_I_SH 0
#define ACTU_D_SH 0

#define ENC3_NOISE_GATE 20
#define ACTU_NOISE_GATE 10

#define ENC3_MIN_SPEED 8
#define ACTU_MIN_SPEED 2

#define ENC3_INT_CAP 10000
#define ACTU_INT_CAP 10000

#define ENC3_PCAP 3000
#define ACTU_PCAP 3000

#define MOTOR3 0
#define MOTOR4 1
// #define MOTOR5 2
// #define MOTOR6 3
// #define MOTOR7 4
// #define MOTOR8 5
#define NUM_MOTORS 2

int16_t pid_ref[NUM_MOTORS], pid_speed[NUM_MOTORS], pid_target[NUM_MOTORS];
int16_t pid_max_speed[] = {1064, 150};
uint8_t pid_complete[NUM_MOTORS];

int16_t enc3_error_last, actu_error_last;
int16_t enc3_pro, actu_pro, enc3_der, actu_der;
int32_t enc3_int = 0, actu_int = 0;
uint8_t pid_on = 0;

void reset_pid(void) {
	uint8_t i;
	
	for(i = 0; i < NUM_MOTORS; i++) {
		pid_ref[i] = pid_target[i] = 0;
		pid_speed[i] = pid_max_speed[i];
		pid_complete[i] = 1;
	}
	
	pid_ref[MOTOR3] = pid_target[MOTOR3] = read_enc();
	pid_ref[MOTOR4] = pid_target[MOTOR4] = read_adc(LIFT_FB);
	enc3_error_last = actu_error_last = enc3_int = actu_int = 0;
}

void update_pid_vals(void) {
	if(!run_pid || !pid_on) return;
	run_pid = 0;
	
	// read encoder values synchronized by TIMER3
	int16_t enc3 = read_enc3_synchro();
	int16_t actu = read_actu_synchro();
	
	// update next position of PID reference
	uint8_t i;
	for(i = 0; i < NUM_MOTORS; i++) {
		     if(pid_target[i] > pid_ref[i]) {
			pid_ref[i] += pid_speed[i];
			if(pid_ref[i] > pid_target[i]) { pid_ref[i] = pid_target[i]; pid_complete[i] = 1; }
		}
		else if(pid_target[i] < pid_ref[i]) {
			pid_ref[i] -= pid_speed[i];
			if(pid_ref[i] < pid_target[i]) { pid_ref[i] = pid_target[i]; pid_complete[i] = 1; }
		}
		else { pid_complete[i] = 1; }
	}
	
	// PID for motor 3 (turn)
	int16_t enc3_pro = pid_ref[MOTOR3] - enc3;
	uint8_t enc3_der_active = 1;
	if(abs(pid_target[MOTOR3] - enc3) < ENC3_NOISE_GATE) enc3_pro = enc3_int = enc3_der_active = 0;
	enc3_int += enc3_pro;
	if(labs(enc3_int) > ENC3_INT_CAP) {
		if(enc3_int < 0) enc3_int = -ENC3_INT_CAP;
		else             enc3_int =  ENC3_INT_CAP;
	}
	int32_t enc3_out = ((int32_t)enc3_pro * ENC3_P) >> ENC3_P_SH;
	enc3_out += (enc3_int * ENC3_I) >> ENC3_I_SH;
	enc3_der = enc3_der_active ? enc3_pro - enc3_error_last : 0;
	//if((enc3_der < 0 && enc3_out > 0) || (enc3_der > 0 && enc3_out < 0)) enc3_der = 0; // HACK
	enc3_out += ((int32_t)enc3_der * ENC3_D) >> ENC3_D_SH;
	enc3_error_last = enc3_pro;
	
	// PID for motor 4 (lift)
	int16_t actu_pro = pid_ref[MOTOR4] - actu;
	uint8_t actu_der_active = 1;
	if(abs(pid_target[MOTOR4] - actu) < ACTU_NOISE_GATE) actu_pro = actu_int = actu_der_active = 0;
	actu_int += actu_pro;
	if(labs(actu_int) > ACTU_INT_CAP) {
		if(actu_int < 0) actu_int = -ACTU_INT_CAP;
		else             actu_int =  ACTU_INT_CAP;
	}
	int32_t actu_out = ((int32_t)actu_pro * ACTU_P) >> ACTU_P_SH;
	actu_out += (actu_int * ACTU_I) >> ACTU_I_SH;
	enc3_der = actu_der_active ? actu_pro - actu_error_last : 0;
	actu_out += ((int32_t)actu_der * ACTU_D) >> ACTU_D_SH;
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

