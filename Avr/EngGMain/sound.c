#define SOUND_LIFT1 0x0000
#define SOUND_LIFT2 0x0001
#define SOUND_SHOOT 0x0002
#define SOUND_MARIO 0x0003
#define SOUND_TRUCK 0x0004

/*
void init_sound(void) {
	SPCR = 1 << SPE | 1 << MSTR | 3 << CPHA | 3 << SPR0; // enable SPI in master mode 3
	nop();
	SPDR = 0xff; // dummy write to set SPI interrupt flag (means last TX complete)
	delay_ms(T_MS * 10);
	set_bit(SNDRST);
	delay_ms(T_MS * 300);
}

void sound_cmd(uint16_t cmd) {
	uint8_t dummy;
	while(~SPSR & 1 << SPIF); // wait for last TX to complete
	dummy = SPDR; // dummy read to clear SPI interrupt flag
	SPDR = cmd >> 8; // push new byte and start TX
	while(~SPSR & 1 << SPIF); // wait for last TX to complete
	dummy = SPDR; // dummy read to clear SPI interrupt flag
	SPDR = cmd & 0xff; // push new byte and start TX
}
*/

void init_sound(void) {
	clr_bit(SNDRST);
	delay_ms(T_MS * 10);
	set_bit(SNDRST);
	delay_ms(T_MS * 300);
}

/*void sound_cmd(uint16_t cmd) {
	uint8_t i;
	set_bit(SNDCLK);
	delay_ms(T_MS * 2);
	clr_bit(SNDCLK);
	delay_us(T_US * 1800);
	
	for (i = 0; i < 16; i++) {
		clr_bit(SNDCLK);
		if (cmd & 0x8000) {
			set_bit(SNDDAT);
		} else {
			clr_bit(SNDDAT);
		}
		cmd <<= 1;
		delay_us(T_US * 200);
		set_bit(SNDCLK);
		delay_us(T_US * 200);
	}
	
	set_bit(SNDDAT);
}*/

void sound_cmd(uint16_t cmd) {
	uint8_t i;
	//clr_bit(SNDCLK);
	//delay_us(T_US * 1800);
	
	for (i = 0; i < 16; i++) {
		clr_bit(SNDCLK);
		if (cmd & 0x8000) {
			set_bit(SNDDAT);
		} else {
			clr_bit(SNDDAT);
		}
		cmd <<= 1;
		delay_us(T_US * 20);
		set_bit(SNDCLK);
		delay_us(T_US * 20);
	}
	
	set_bit(SNDDAT);
}
