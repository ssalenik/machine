#define ENC3_P  30
#define ENC3_I  50
#define ENC3_D 250

#define ACTU_P  60
#define ACTU_I 150
#define ACTU_D 350

#define ENC3_P_SH 0
#define ENC3_I_SH 8
#define ENC3_D_SH 8

#define ACTU_P_SH 0
#define ACTU_I_SH 8
#define ACTU_D_SH 8

#define ENC3_NOISE_GATE 20
#define ACTU_NOISE_GATE 10

//#define ENC3_MIN_SPEED 8
//#define ACTU_MIN_SPEED 2

#define ENC3_INT_CAP 1000000L
#define ACTU_INT_CAP 1000000L

#define ENC3_PCAP 3500
#define ACTU_PCAP 3500

#define MOTOR3 0
#define MOTOR4 1
// #define MOTOR5 2
// #define MOTOR6 3
// #define MOTOR7 4
// #define MOTOR8 5
#define NUM_MOTORS 2

int16_t pid_ref[NUM_MOTORS], pid_speed[NUM_MOTORS], pid_target[NUM_MOTORS];
int16_t pid_max_speed[] = {1064, 150};
uint8_t pid_complete[NUM_MOTORS], ref_complete[NUM_MOTORS];

uint32_t enc3_last_time = 0, actu_last_time = 0;
int16_t  enc3_last_val = 0, actu_last_val = 0, enc3_pro, actu_pro;
int32_t  enc3_der, actu_der, enc3_int = 0, actu_int = 0;
uint8_t  pid_on = 1;

void reset_pid(void) {
	uint8_t i;
	
	for(i = 0; i < NUM_MOTORS; i++) {
		pid_ref[i] = pid_target[i] = 0;
		pid_speed[i] = pid_max_speed[i];
		pid_complete[i] = ref_complete[i] = 1;
	}
	
	pid_ref[MOTOR3] = pid_target[MOTOR3] = read_enc();
	pid_ref[MOTOR4] = pid_target[MOTOR4] = read_actu();
	enc3_last_time = actu_last_time = uptime32();
	cli();
	enc3_last_val = V_enc3_sync_val;
	actu_last_val = V_actu_sync_val;
	sei();
	enc3_int = actu_int = 0;
}

void update_pid_vals(void) {
	if(!run_pid) return;
	run_pid = 0;
	
	// read encoder values and their timestamps synchronized by TIMER3
	cli();
	uint32_t enc3_time = V_enc3_sync_time;
	uint32_t actu_time = V_actu_sync_time;
	int16_t  enc3_val  = V_enc3_sync_val;
	int16_t  actu_val  = V_actu_sync_val;
	sei();
	
	// update next position of PID reference
	uint8_t i;
	for(i = 0; i < NUM_MOTORS; i++) {
		     if(pid_target[i] > pid_ref[i]) {
			pid_ref[i] += pid_speed[i];
			if(pid_ref[i] > pid_target[i]) { pid_ref[i] = pid_target[i]; ref_complete[i] = 1; }
			// FIXME: try not to set pid_ref[i] = pid_target[i]
		}
		else if(pid_target[i] < pid_ref[i]) {
			pid_ref[i] -= pid_speed[i];
			if(pid_ref[i] < pid_target[i]) { pid_ref[i] = pid_target[i]; ref_complete[i] = 1; }
			// FIXME: try not to set pid_ref[i] = pid_target[i]
		}
		else { ref_complete[i] = 1; }
	}
	
	// calculate speed of motor 3 (turn)
	if(enc3_val == enc3_last_val) { enc3_der = 0; }
	else if(enc3_last_val < enc3_val) {
		enc3_der = (int32_t)(enc3_val - enc3_last_val) * TIMER32_FREQ / (enc3_time - enc3_last_time);
	}
	else {
		enc3_der = -((int32_t)(enc3_last_val - enc3_val) * TIMER32_FREQ / (enc3_time - enc3_last_time));
	}
	
	enc3_last_time = enc3_time;
	enc3_last_val  = enc3_val;
	
	// calculate speed of motor 4 (lift)
	if(actu_val == actu_last_val) { actu_der = 0; }
	else if(actu_last_val < actu_val) {
		actu_der = (int32_t)(actu_val - actu_last_val) * TIMER32_FREQ / (actu_time - actu_last_time);
	}
	else {
		actu_der = -((int32_t)(actu_last_val - actu_val) * TIMER32_FREQ / (actu_time - actu_last_time));
	}
	
	actu_last_time = actu_time;
	actu_last_val  = actu_val;
	
	if(!pid_on) return;
	
	// PID for motor 3 (turn)
	int16_t enc3_pro = pid_ref[MOTOR3] - enc3_val;
	//uint8_t enc3_der_active = 1;
	if(abs(pid_target[MOTOR3] - enc3_val) < ENC3_NOISE_GATE) { enc3_pro = enc3_int = 0; pid_complete[MOTOR3] = 1; }
	enc3_int += enc3_pro;
	if(labs(enc3_int) > ENC3_INT_CAP) { enc3_int = (enc3_int < 0) ? -ENC3_INT_CAP : ENC3_INT_CAP; }
	int32_t enc3_out = ((int32_t)enc3_pro * ENC3_P) >> ENC3_P_SH;
	enc3_out += (enc3_int * ENC3_I) >> ENC3_I_SH;
	//enc3_der = enc3_der_active ? enc3_pro - enc3_error_last : 0;
	//if((enc3_der < 0 && enc3_out > 0) || (enc3_der > 0 && enc3_out < 0)) enc3_der = 0; // HACK
	enc3_out -= (enc3_der * ENC3_D) >> ENC3_D_SH;
	
	// PID for motor 4 (lift)
	int16_t actu_pro = pid_ref[MOTOR4] - actu_val;
	//uint8_t actu_der_active = 1;
	if(abs(pid_target[MOTOR4] - actu_val) < ACTU_NOISE_GATE) { actu_pro = actu_int = 0; pid_complete[MOTOR4] = 1; }
	actu_int += actu_pro;
	if(labs(actu_int) > ACTU_INT_CAP) { actu_int = (actu_int < 0) ? -ACTU_INT_CAP : ACTU_INT_CAP; }
	int32_t actu_out = ((int32_t)actu_pro * ACTU_P) >> ACTU_P_SH;
	actu_out += (actu_int * ACTU_I) >> ACTU_I_SH;
	//enc3_der = actu_der_active ? actu_pro - actu_error_last : 0;
	actu_out -= (actu_der * ACTU_D) >> ACTU_D_SH;
	
	// control of motor 3 (turn)
	if(enc3_out < 0) { motor3_rev(); }
	else             { motor3_fwd(); }
	enc3_out = labs(enc3_out);
	if(enc3_out > ENC3_PCAP) { enc3_out = ENC3_PCAP; }
	set_speed_3((int16_t)enc3_out);
	
	// control of motor 4 (lift)
	if(actu_out < 0) { motor4_rev(); }
	else             { motor4_fwd(); }
	actu_out = labs(actu_out);
	if(actu_out > ACTU_PCAP) { actu_out = ACTU_PCAP; }
	set_speed_4((int16_t)actu_out);
}

