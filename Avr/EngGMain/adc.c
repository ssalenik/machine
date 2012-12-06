void init_adc(void) {
	ADMUX  |= 1 << REFS0; // VREF = AVCC
	ADCSRA |= 7 << ADPS0; // prescaler = 128 to get ADC clock = 156.25 kHz
	ADCSRA |= 1 << ADEN;  // enable ADC
}

uint16_t read_adc(uint8_t channel) {
	ADMUX = (ADMUX & 0xf8) | (channel & 0x07); // select channel
	ADCSRA |= 1 << ADSC; // start conversion
	while(ADCSRA & 1 << ADSC); // wait for conversion to finish
	return ADCW;
}

