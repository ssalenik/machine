uint8_t nib2ascii(uint8_t hex) {
	hex |= 0x30;
	if(hex > '9') hex += 7;
	return hex;
}

uint8_t ascii2nib(uint8_t ascii) {
	if(ascii > 0x40) {
		ascii |= 0x20; // lowercase
		ascii -= 0x27;
	}
	ascii &= 0x0f;
	return ascii;
}

uint8_t htoa(uint8_t MSB, uint8_t LSB) {
	return (ascii2nib(MSB) << 4) | ascii2nib(LSB);
}

void atoh(uint8_t data, uint8_t* array) {
	array[0] = nib2ascii((data >> 4) & 0x0f);
	array[1] = nib2ascii(data & 0x0f);
}

uint8_t isHex(uint8_t ascii) {
	if(ascii >= '0' && ascii <= '9') return 1;
	ascii |= 0x20; // lowercase
	if(ascii >= 'a' && ascii <= 'f') return 1;
	return 0;
}

