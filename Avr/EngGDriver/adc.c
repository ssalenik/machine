volatile uint16_t p_LsensADC, p_RsensADC;
uint8_t adc_channel;

void initADC() {
    ADMUX   = 1 << REFS0; // VREF = AVCC, channel 0
    ADCSRA |= 7 << ADPS0; // prescaler = 128 to get ADC clock = 156.25 kHz
    ADCSRA |= 1 << ADEN;  // enable ADC
    
    delay_ms(T_MS * 100); // wait for capacitors to charge
    
    ADCSRA |= 1 << ADIF; // clear ADC interrupt flag
    ADCSRA |= 1 << ADIE; // enable ADC interrupt
    ADCSRA |= 1 << ADSC; // start conversion
}

uint16_t readADC(uint8_t channel) {
    uint16_t val;
    
    if (channel) {
        cli();
        val = p_RsensADC;
        sei();
    } else {
        cli();
        val = p_LsensADC;
        sei();
    }
    
    return val;
}

ISR(ADC_vect) {
    uint16_t val = ADCW; // read result
    
    if (adc_channel) {
        p_RsensADC = val;
        ADMUX = 1 << REFS0 | 0 << MUX0; // switch channel
        adc_channel = 0;
    } else {
        p_LsensADC = val;
        ADMUX = 1 << REFS0 | 1 << MUX0; // switch channel
        adc_channel = 1;
    }
    
    ADCSRA |= 1 << ADSC; // start conversion
}

