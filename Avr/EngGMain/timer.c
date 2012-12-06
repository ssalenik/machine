volatile uint32_t V_uptime;

void init_timer(void) {
	// init. TIMER0
	TCCR0A = 2 << COM0A0 | 2 << COM0B0; // clear on compare match on all channels
	TCCR0B = 4 << CS00; // start TIMER0 with prescaler = 256
	
	// init. TIMER2
	TCCR2A = 2 << COM2A0 | 2 << COM2B0; // clear on compare match on all channels
	TCCR2B = 6 << CS20; // start TIMER2 with prescaler = 256
	
	// init. TIMER1
	TCCR1B |= 1 << WGM13; // phase & frequency correct PWM (mode 8)
	ICR1 = F_CPU / 800; // 25000 @ 20 MHz, 10000 @ 8 MHz
	TCNT1 = ICR1 >> 1; // set timer to middle to prevent output pulse
	OCR1A = PWM_OFFSET; // set output to 0 degrees on channel A
	OCR1B = PWM_OFFSET; // set output to 0 degrees on channel B
	TCCR1B |= 2 << CS10; // set prescaler to 8 (start TIMER1)
	TCCR1A |= 0b10100000; // enable PWM1A, PWM1B
	
	// init. TIMER3
	
}

uint32_t uptime(void) {
	cli();
	uint32_t time = V_uptime;
	sei();
	return time;
}

void servo1(uint8_t degrees) { OCR1A = degrees * 20; }
void servo2(uint8_t degrees) { OCR1B = degrees * 20; }

// TIMER0 interrupt
SIGNAL(TIMER0_COMPA_vect) {
	static uint8_t repeat = 0;
	repeat++;
	if(repeat == 10) {
		repeat = 0;
		V_uptime++;
	}
}

