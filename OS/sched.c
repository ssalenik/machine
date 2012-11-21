#define _AUTORELOAD (F_CPU / 64000 - 1)

volatile unsigned long rtc_time = 0, _RTC_T0, _RTC_T1, _RTC_T2, _RTC_T3, _RTC_T4, _RTC_T5, _RTC_T6, _RTC_T7;
volatile char _RTC_EN = 0, _RTC_EQ = 0;

void (*_RTC_A0)(void);
void (*_RTC_A1)(void);
void (*_RTC_A2)(void);
void (*_RTC_A3)(void);
void (*_RTC_A4)(void);
void (*_RTC_A5)(void);
void (*_RTC_A6)(void);
void (*_RTC_A7)(void);

/**
 *  rtc_set(pointer to function to schedule, time delay in ms)
 *  Schedules a void function for future execution.
 *  The same function can be scheduled more than once.
 */
void rtc_set(void (*func)(void), unsigned long time) {
	time += rtc_time;
	
	// are we already in an interrupt?
	char gie_set = 0;
	if(SREG & 1 << SREG_I) gie_set = 1;
	
	if(gie_set) cli(); // if so, disable interrupts
	
	     if(~_RTC_EN & 1 << 0) { _RTC_EN |= 1 << 0; _RTC_T0 = time; _RTC_A0 = func; }
	else if(~_RTC_EN & 1 << 1) { _RTC_EN |= 1 << 1; _RTC_T1 = time; _RTC_A1 = func; }
	else if(~_RTC_EN & 1 << 2) { _RTC_EN |= 1 << 2; _RTC_T2 = time; _RTC_A2 = func; }
	else if(~_RTC_EN & 1 << 3) { _RTC_EN |= 1 << 3; _RTC_T3 = time; _RTC_A3 = func; }
	else if(~_RTC_EN & 1 << 4) { _RTC_EN |= 1 << 4; _RTC_T4 = time; _RTC_A4 = func; }
	else if(~_RTC_EN & 1 << 5) { _RTC_EN |= 1 << 5; _RTC_T5 = time; _RTC_A5 = func; }
	else if(~_RTC_EN & 1 << 6) { _RTC_EN |= 1 << 6; _RTC_T6 = time; _RTC_A6 = func; }
	else if(~_RTC_EN & 1 << 7) { _RTC_EN |= 1 << 7; _RTC_T7 = time; _RTC_A7 = func; }
	
	if(gie_set) sei(); // if so, enable interrupts
}

/**
 *  rtc_clr(pointer to function to remove from scheduling list)
 *  Cancels a scheduled void function by name.
 *  If more than 1 instance is scheduled, all instances are cancelled.
 */
void rtc_clr(void (*func)(void)) {
	// are we already in an interrupt?
	char gie_set = 0;
	if(SREG & 1 << SREG_I) gie_set = 1;
	
	if(gie_set) cli(); // if so, disable interrupts
	
	if(_RTC_EN & 1 << 0 && _RTC_A0 == func) { _RTC_EN &= ~(1 << 0); _RTC_EQ &= ~(1 << 0); }
	if(_RTC_EN & 1 << 1 && _RTC_A1 == func) { _RTC_EN &= ~(1 << 1); _RTC_EQ &= ~(1 << 1); }
	if(_RTC_EN & 1 << 2 && _RTC_A2 == func) { _RTC_EN &= ~(1 << 2); _RTC_EQ &= ~(1 << 2); }
	if(_RTC_EN & 1 << 3 && _RTC_A3 == func) { _RTC_EN &= ~(1 << 3); _RTC_EQ &= ~(1 << 3); }
	if(_RTC_EN & 1 << 4 && _RTC_A4 == func) { _RTC_EN &= ~(1 << 4); _RTC_EQ &= ~(1 << 4); }
	if(_RTC_EN & 1 << 5 && _RTC_A5 == func) { _RTC_EN &= ~(1 << 5); _RTC_EQ &= ~(1 << 5); }
	if(_RTC_EN & 1 << 6 && _RTC_A6 == func) { _RTC_EN &= ~(1 << 6); _RTC_EQ &= ~(1 << 6); }
	if(_RTC_EN & 1 << 7 && _RTC_A7 == func) { _RTC_EN &= ~(1 << 7); _RTC_EQ &= ~(1 << 7); }
	
	if(gie_set) sei(); // if so, enable interrupts
}

inline void rtc_init(void) {
	TCCR0A = 1 << WGM01;	// auto-reload TIMER0
	OCR0A  = _AUTORELOAD;	// auto-reload value
	TIMSK0 = 1 << OCIE0A;	// enable TIMER0 interrupt
	TCCR0B = 1 << CS01 | 1 << CS00;	// clock /64
	
	sei();	// enable interrupts
	
	while(1) {
		// throttling
		if(SMCR & 1 << SE) sleep_cpu();
		SMCR |= 1 << SE;
		
		if(_RTC_EQ & 1 << 0) { (*_RTC_A0)(); _RTC_EN &= ~(1 << 0); _RTC_EQ &= ~(1 << 0); }
		if(_RTC_EQ & 1 << 1) { (*_RTC_A1)(); _RTC_EN &= ~(1 << 1); _RTC_EQ &= ~(1 << 1); }
		if(_RTC_EQ & 1 << 2) { (*_RTC_A2)(); _RTC_EN &= ~(1 << 2); _RTC_EQ &= ~(1 << 2); }
		if(_RTC_EQ & 1 << 3) { (*_RTC_A3)(); _RTC_EN &= ~(1 << 3); _RTC_EQ &= ~(1 << 3); }
		if(_RTC_EQ & 1 << 4) { (*_RTC_A4)(); _RTC_EN &= ~(1 << 4); _RTC_EQ &= ~(1 << 4); }
		if(_RTC_EQ & 1 << 5) { (*_RTC_A5)(); _RTC_EN &= ~(1 << 5); _RTC_EQ &= ~(1 << 5); }
		if(_RTC_EQ & 1 << 6) { (*_RTC_A6)(); _RTC_EN &= ~(1 << 6); _RTC_EQ &= ~(1 << 6); }
		if(_RTC_EQ & 1 << 7) { (*_RTC_A7)(); _RTC_EN &= ~(1 << 7); _RTC_EQ &= ~(1 << 7); }
	}
}

inline void rtc_intr(void) {
	if(_RTC_EN & 1 << 0 && rtc_time == _RTC_T0) { _RTC_EQ |= 1 << 0; }
	if(_RTC_EN & 1 << 1 && rtc_time == _RTC_T1) { _RTC_EQ |= 1 << 1; }
	if(_RTC_EN & 1 << 2 && rtc_time == _RTC_T2) { _RTC_EQ |= 1 << 2; }
	if(_RTC_EN & 1 << 3 && rtc_time == _RTC_T3) { _RTC_EQ |= 1 << 3; }
	if(_RTC_EN & 1 << 4 && rtc_time == _RTC_T4) { _RTC_EQ |= 1 << 4; }
	if(_RTC_EN & 1 << 5 && rtc_time == _RTC_T5) { _RTC_EQ |= 1 << 5; }
	if(_RTC_EN & 1 << 6 && rtc_time == _RTC_T6) { _RTC_EQ |= 1 << 6; }
	if(_RTC_EN & 1 << 7 && rtc_time == _RTC_T7) { _RTC_EQ |= 1 << 7; }
	
	rtc_time++;
	SMCR &= ~(1 << SE); // for throttling
}

