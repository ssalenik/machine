#include <avr/io.h>
//#include "hex.h"

char nib2ascii(char hex) {
	hex |= 0x30;
	if(hex > '9') hex += 7;
	return hex;
}

uint8_t isHex(char ascii) {
	if (ascii >= '0' && ascii <= '9') return 1;
	ascii |= 0x20;
	if (ascii >= 'a' && ascii <= 'f') return 1;
	return 0;
}

char ascii2nib(char ascii) {
	if(ascii > 0x40) {
		ascii |= 0x20; // lowercase
		ascii -= 0x27;
	}
	ascii &= 0x0f;
	return ascii;
}

char htoi(char MSB, char LSB) {
	return (ascii2nib(MSB) << 4) | ascii2nib(LSB);
}

void atoh(char data, char* array) {
	array[0] = nib2ascii((data >> 4) & 0x0f);
	array[1] = nib2ascii(data & 0x0f);
}
