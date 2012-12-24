uint8_t last_qio, last_qint;

int16_t read_enc(void) {
	cli();
	int16_t pos = V_encoder;
	sei();
	return pos;
}

void init_enc(void) {
	last_qio = last_qint = PINB; // initialize quadrature state
	EICRA = 1 << ISC20;
	EIMSK = 1 << INT2; // enable interrupt on QINT (both edges)
	PCMSK1 = 1 << PCINT9;
	PCICR  = 1 << PCIE1; // enable interrupt on QIO (both edges)
}

SIGNAL(INT2_vect) {
	last_qio = Q3_PIN;
	
	if(last_qio & 1 << Q3_IO) {
		if(last_qint & 1 << Q3_INT) { V_encoder--; }
		else                        { V_encoder++; }
	}
	else {
		if(last_qint & 1 << Q3_INT) { V_encoder++; }
		else                        { V_encoder--; }
	}
	
	V_encoder_time = V_uptime32;
	
	EIMSK = 0;
	PCICR = 1 << PCIE1;
}

SIGNAL(PCINT1_vect) {
	last_qint = Q3_PIN;
	
	if(last_qint & 1 << Q3_INT) {
		if(last_qio & 1 << Q3_IO) { V_encoder++; }
		else                      { V_encoder--; }
	}
	else {
		if(last_qio & 1 << Q3_IO) { V_encoder--; }
		else                      { V_encoder++; }
	}
	
	V_encoder_time = V_uptime32;
	
	PCICR = 0;
	EIMSK = 1 << INT2;
}

