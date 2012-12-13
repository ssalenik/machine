volatile uint32_t V_uptime = 0;
volatile int16_t V_enc3_synchro = 0, V_actu_synchro = 0;
volatile uint8_t run_pid = 0;

uint32_t uptime(void)           { cli(); uint32_t time = V_uptime; sei(); return time; }

int16_t read_enc3_synchro(void) { cli(); int16_t val = V_enc3_synchro; sei(); return val; }
int16_t read_actu_synchro(void) { cli(); int16_t val = V_actu_synchro; sei(); return val; }

// set PWM for turn and lift motors (100% = 4000)
#define MAXTIMER 4000

void set_speed_3(uint16_t speed) {
	if(speed > MAXTIMER) speed = MAXTIMER;
	OCR1A = speed;
}

void set_speed_4(uint16_t speed) {
	if(speed > MAXTIMER) speed = MAXTIMER;
	OCR1B = MAXTIMER - speed;
}

// set angle of servos
#define SERVO_OFFSET 49
#define SERVO_FACTOR 193
// these can also be set individually for each servo below
void servo5(uint8_t degrees) { OCR0A = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; }
void servo6(uint8_t degrees) { OCR0B = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; }
void servo7(uint8_t degrees) { OCR2A = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; }
void servo8(uint8_t degrees) { OCR2B = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; }

void init_timer(void) {
	// init. TIMER0 & TIMER2
	TCCR0A = 2 << COM0A0 | 2 << COM0B0; // clear on compare match on all channels
	TCCR2A = 2 << COM2A0 | 2 << COM2B0;
	
	// init. servo positions to avoid 0 in compare match registers
	servo5(0);
	servo6(0);
	servo7(0);
	servo8(0);
	
	// init. TIMER1
	TCCR1B = 1 << WGM13; // phase & frequency correct PWM (mode 8)
	ICR1 = MAXTIMER; // PWM output frequency = 2.500 kHz
	TCNT1 = MAXTIMER >> 1; // set timer to middle to prevent output pulse
	OCR1A = 0; // set output to 0 on both channels
	OCR1B = MAXTIMER;
	TCCR1B |= 1 << CS10; // start TIMER1 with prescaler = 1
	TCCR1A |= 2 << COM1A0 | 3 << COM1B0; // enable PWM1A, PWM1B in opposite phase
	
	// init. TIMER3
	TCCR3B = 1 << WGM32; // clear timer on compare match (mode 4)
	OCR3A = 2500 - 1; // 1 ms interval
	TIMSK3 = 1 << OCIE3A; // enable TIMER3 output compare match interrupt
	TCCR3B |= 2 << CS30; // start TIMER3 with prescaler = 8
}

// TIMER3 interrupt
SIGNAL(TIMER3_COMPA_vect) {
	static uint8_t servo_period = 0;
	V_uptime++;
	servo_period++;
	if(servo_period == 20) {
		servo_period = 0;
		
		// servo outputs
		TCCR0B = 0; // stop TIMER0
		TCCR2B = 0; // stop TIMER2
		TCNT0 = 0;  // reset TIMER0
		TCNT2 = 0;  // reset TIMER2
		TCCR0A = 3 << COM0A0 | 3 << COM0B0; // set on compare match on all channels
		TCCR2A = 3 << COM2A0 | 3 << COM2B0;
		TCCR0B = 1 << FOC0A  | 1 << FOC0B;  // force compare match
		TCCR2B = 1 << FOC2A  | 1 << FOC2B;
		TCCR0A = 2 << COM0A0 | 2 << COM0B0; // clear on compare match on all channels
		TCCR2A = 2 << COM2A0 | 2 << COM2B0;
		TCCR0B = 4 << CS00; // start TIMER0 with prescaler = 256 (max pulse = 3.264 ms)
		TCCR2B = 6 << CS20; // start TIMER2 with prescaler = 256 (max pulse = 3.264 ms)
		
		// precise intervals for encoder sampling
		V_enc3_synchro = V_encoder;
		V_actu_synchro = adc_sum[LIFT_FB] >> ADC_FILTER_POWER;
		run_pid = 1;
	}
}

