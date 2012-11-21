#define PWM_OFFSET (F_CPU / 16000)
#define PWM_DEGREE ((PWM_OFFSET + 90) / 180)

volatile uint32_t V_uptime;
volatile uint16_t V_interval;

void init_timer(void) {
	// init. TIMER0
	TCCR0A |= 1 << WGM01; // clear timer on compare match (mode 2)
	OCR0A   = F_CPU / 80000 - 1; // 0.1 ms interval (will be multiplied by 10)
	TIMSK0 |= 1 << OCIE0A; // enable TIMER3 output compare match interrupt
	TCCR0B |= 2 << CS00; // start TIMER3 with prescaler = 8 (mode 2)
	
	// init. TIMER1
	TCCR1B |= 1 << WGM13; // phase & frequency correct PWM (mode 8)
	ICR1 = F_CPU / 800; // 25000 @ 20 MHz, 10000 @ 8 MHz
	TCNT1 = ICR1 >> 1; // set timer to middle to prevent output pulse
	OCR1A = PWM_OFFSET; // set output to 0 degrees on channel A
	OCR1B = PWM_OFFSET; // set output to 0 degrees on channel B
	TCCR1B |= 2 << CS10; // set prescaler to 8 (start TIMER1)
	TCCR1A |= 0b10100000; // enable PWM1A, PWM1B
}

uint32_t uptime(void) {
	cli();
	uint32_t time = V_uptime;
	sei();
	return time;
}

//void servo1(uint8_t degrees) { OCR1A = degrees * PWM_DEGREE + PWM_OFFSET; }
//void servo2(uint8_t degrees) { OCR1B = degrees * PWM_DEGREE + PWM_OFFSET; }
void servo1(uint8_t degrees) { OCR1A = degrees * 20; }
void servo2(uint8_t degrees) { OCR1B = degrees * 20; }

/*
void sleep(uint16_t seconds) {
	uint32_t target = uptime() + (uint32_t)seconds * 1000;
	while(uptime() < target);
}
*/

void test_interval_start(void) {
	TCCR3B &= 0xf8; // stop TIMER3
	TCNT3 = 0;
	TIFR3 = 1 << TOV3; // clear TIMER3 overflow flag
	TCCR3B |= 1 << CS30; // start TIMER3 with prescaler = 1 (mode 1)
}

uint16_t test_interval_stop(void) {
	uint16_t result;
	TCCR3B &= 0xf8; // stop TIMER3
	result = (TIFR3 & 1 << TOV3) ? 0xffff : TCNT3;
	TCNT3 = 0;
	TIFR3 = 1 << TOV3; // clear TIMER3 overflow flag
	return result;
}

uint16_t test_interval(void) {
	cli();
	uint16_t result = V_interval;
	sei();
	return result;
}

// TIMER0 interrupt
SIGNAL(TIMER0_COMPA_vect) {
	static uint8_t repeat = 0;
	repeat++;
	if(repeat == 10) {
		repeat = 0;
		V_uptime++;
	}
}

