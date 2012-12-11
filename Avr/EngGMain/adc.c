#define ADC_NUM_CHANNELS 6
#define ADC_FILTER_POWER 6

volatile uint16_t adc_sum[ADC_NUM_CHANNELS];
volatile uint16_t adc_filter[ADC_NUM_CHANNELS][1 << ADC_FILTER_POWER];
uint8_t adc_channel = 0, adc_sample = 0;

void init_adc(void) {
	ADMUX   = 1 << REFS0; // VREF = AVCC, channel 0
	ADCSRA |= 7 << ADPS0; // prescaler = 128 to get ADC clock = 156.25 kHz
	ADCSRA |= 1 << ADEN | 1 << ADIE; // enable ADC and its interrupt
	ADCSRA |= 1 << ADSC; // start conversion
}

/*
uint16_t read_adc(uint8_t channel) {
	ADMUX = (ADMUX & 0xf8) | (channel & 0x07); // select channel
	ADCSRA |= 1 << ADSC; // start conversion
	while(ADCSRA & 1 << ADSC); // wait for conversion to finish
	return ADCW;
}
*/

uint16_t read_adc(uint8_t channel) {
	cli();
	uint16_t value = adc_sum[channel] >> ADC_FILTER_POWER;
	sei();
	return value;
}

SIGNAL(ADC_vect) {
	uint16_t sum = adc_sum[adc_channel];
	sum -= adc_filter[adc_channel][adc_sample];
	uint16_t val = ADCW; // read result
	adc_filter[adc_channel][adc_sample] = val;
	sum += val;
	adc_sum[adc_channel] = sum;
	
	adc_channel++; // next channel
	if(adc_channel == ADC_NUM_CHANNELS) {
		adc_channel = 0;
		adc_sample++;
		adc_sample &= ((1 << ADC_FILTER_POWER) - 1);
	}
	
	ADMUX = (ADMUX & 0b11100000) | adc_channel; // set channel
	ADCSRA |= 1 << ADSC; // start conversion
}

