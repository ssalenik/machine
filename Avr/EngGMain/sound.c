void init_sound(void) {
	SPCR = 1 << SPE | 1 << MSTR | 3 << CPHA | 3 << SPR0; // enable SPI in master mode 3
	set_ddr(SNDRST);
	SPDR = 0xff; // dummy write to set SPI interrupt flag (means last TX complete)
	delay_ms(T_MS * 5);
	clr_ddr(SNDRST);
	delay_ms(T_MS * 300);
}

void sound_cmd(uint8_t cmd) {
	uint8_t dummy;
	while(~SPSR & 1 << SPIF); // wait for last TX to complete
	dummy = SPDR; // dummy read to clear SPI interrupt flag
	SPDR = cmd; // push new byte and start TX
}

