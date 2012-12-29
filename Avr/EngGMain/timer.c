#define TIMER32_FREQ 32000

volatile uint32_t V_uptime = 0, V_uptime32 = 0;
volatile uint32_t V_encoder_time = 0, V_actuator_time = 0, V_enc3_sync_time = 0, V_actu_sync_time = 0;
volatile  int16_t V_enc3_sync_val = 0, V_actu_sync_val = 0, V_encoder = 2128;
volatile uint16_t V_actuator = 0;
volatile uint8_t  run_pid = 0;

uint32_t uptime(void)   { cli(); uint32_t time = V_uptime;   sei(); return time; }
uint32_t uptime32(void) { cli(); uint32_t time = V_uptime32; sei(); return time; }

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
#define SERVO_DELAY  400
// these can also be set individually for each servo below
void servo5(uint8_t degrees) { OCR0A = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; } // ball
void servo6(uint8_t degrees) { OCR0B = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; } // laser
void servo7(uint8_t degrees) { OCR2A = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; } // claw v.
void servo8(uint8_t degrees) { OCR2B = (((uint16_t)degrees * SERVO_FACTOR) >> 8) + SERVO_OFFSET; } // claw h.
// servo preset values
void cannon_reload(void) { servo5(0);   }
void cannon_shoot(void)  { servo5(50);  }
void laser_init(void)    { servo6(4);   }
void laser_mid(void)     { servo6(58);  }
void laser_full(void)    { servo6(96);  }
void claw_up(void)       { servo7(96);  }
void claw_down(void)     { servo7(160); }
void claw_open(void)     { servo8(0);   }
void claw_close(void)    { servo8(160); }

void init_timer(void) {
	// init. TIMER0 & TIMER2
	TCCR0A = 2 << COM0A0 | 2 << COM0B0; // clear on compare match on all channels
	TCCR2A = 2 << COM2A0 | 2 << COM2B0;
	
	// init. servo positions to avoid 0 in compare match registers
	cannon_reload();
	laser_init();
	claw_up();
	claw_open();
	
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
	OCR3A = 625 - 1; // 31.25 us (1/32 ms) interval
	TIMSK3 = 1 << OCIE3A; // enable TIMER3 output compare match interrupt
	TCCR3B |= 1 << CS30; // start TIMER3 with prescaler = 1
}

// TIMER3 interrupt
SIGNAL(TIMER3_COMPA_vect) {
	static uint8_t repeat = 0, servo_period = 0;
	
	V_uptime32++;
	repeat++;
	if(repeat & (32 - 1)) return;
	
	V_uptime++;
	servo_period++;
	if(servo_period != 20) return;
	servo_period = 0;
	
	// servo outputs
	TCCR0B = 0; // stop TIMER0
	TCCR2B = 0; // stop TIMER2
	TCNT0  = 0; // reset TIMER0
	TCNT2  = 0; // reset TIMER2
	TCCR0A = 3 << COM0A0 | 3 << COM0B0; // set on compare match on all channels
	TCCR2A = 3 << COM2A0 | 3 << COM2B0;
	TCCR0B = 1 << FOC0A  | 1 << FOC0B;  // force compare match
	TCCR2B = 1 << FOC2A  | 1 << FOC2B;
	TCCR0A = 2 << COM0A0 | 2 << COM0B0; // clear on compare match on all channels
	TCCR2A = 2 << COM2A0 | 2 << COM2B0;
	TCCR0B = 4 << CS00; // start TIMER0 with prescaler = 256 (max pulse = 3.264 ms)
	TCCR2B = 6 << CS20; // start TIMER2 with prescaler = 256 (max pulse = 3.264 ms)
	
	// precise intervals for encoder and actuator sampling
	V_enc3_sync_time = V_encoder_time;
	V_actu_sync_time = V_actuator_time;
	V_enc3_sync_val  = V_encoder;
	V_actu_sync_val  = V_actuator;
	run_pid = 1;
}

