#define ACC_ADDR 0x32
#define I2C_MASK 0xf8
#define I2C_MT_SLA_ACK 0x18

// calibration values
#define  OFFX	-0.002116
#define  OFFY	-0.000814
#define  OFFZ	-0.007650
#define  SENSX	 0.989746
#define  SENSY	 1.003418
#define  SENSZ	 1.002441
#define  CXY	-0.010853
#define  CXZ	 0.036507
#define  CYZ	-0.013625
#define  CYX	-0.026277
#define  CZX	-0.049196
#define  CZY	 0.026790

#define unitG	0x4000
#define iterOrder 6

union { uint8_t u8[6]; int16_t s16[3]; } acc_raw;

void i2c_cmd(uint8_t cmd) {
	uint8_t dummy;
	TWCR = cmd; // perform command code
	while(~TWCR & 1 << TWINT); // wait for completion
	dummy = TWSR; // read status (required)
}

void i2c_addr(uint8_t addr, uint8_t reg) {
	do {
		i2c_cmd(1 << TWINT | 1 << TWEN | 1 << TWSTA); // send start bit
		
		TWDR = addr; // write device address
		TWCR = 1 << TWINT | 1 << TWEN; // send device address
		while(~TWCR & 1 << TWINT); // wait for completion
	} while((TWSR & I2C_MASK) != I2C_MT_SLA_ACK); // if no ACK, retry...
	
	TWDR = reg; // write register address
	i2c_cmd(1 << TWINT | 1 << TWEN); // send register address
}

void i2c_wr(uint8_t addr, uint8_t reg, uint8_t data) {
	i2c_addr(addr, reg); // write mode
	
	TWDR = data; // write data
	i2c_cmd(1 << TWINT | 1 << TWEN); // send data
	
	TWCR = 1 << TWINT | 1 << TWEN | 1 << TWSTO; // send stop bit
}

void i2c_rd(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t num_bytes) {
	uint8_t i;
	
	i2c_addr(addr, reg | 0x80); // write mode, regsiter address auto-increment
	
	i2c_cmd(1 << TWINT | 1 << TWEN | 1 << TWSTA); // send repeated start
	
	TWDR = addr | 0x01; // write device address for receiving
	i2c_cmd(1 << TWINT | 1 << TWEN); // send device address
	
	for(i = 0; i < num_bytes - 1; i++) {
		i2c_cmd(1 << TWINT | 1 << TWEN | 1 << TWEA); // receive data byte & send ACK
		*buffer = TWDR; // read data
		buffer++; // next address
	}
	
	i2c_cmd(1 << TWINT | 1 << TWEN | 0 << TWEA); // receive last data byte
	*buffer = TWDR; // read data
	
	TWCR = 1 << TWINT | 1 << TWEN | 1 << TWSTO; // send stop bit
}

void init_acc(void) {
	// init. I2C module
	TWBR = 17; // 400 kHz
	
	// init. accelerometer
	//i2c_wr(ACC_ADDR, 0x20, 0xc7); // XYZ axis ON, DR = 50Hz, ODR = 10Hz, LPF ON
	i2c_wr(ACC_ADDR, 0x20, 0x27); // XYZ axis ON, DR = 50Hz, ODR = 50Hz, LPF OFF
	i2c_wr(ACC_ADDR, 0x23, 0x80); // register update lock ON, FS = 2g (default)
}

void acc_update(void) {
	static int16_t acc[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
	static uint8_t idx = 0;
	int16_t sum;
	uint8_t axis, i;
	
	i2c_rd(ACC_ADDR, 0x28, acc_raw.u8, 6);
	
	// data filter
	for(axis = 0; axis < 3; axis++) {
		acc[axis][idx] = acc_raw.s16[axis] >> 2;
		sum = 2;
		for(i = 0; i < 4; i++) { sum += acc[axis][i]; }
		acc_raw.s16[axis] = sum;
	}
	
	idx++;
	idx &= 3; // circular buffer
}

void read_acc_values(float *values) {
	float acc[3], accPreIter[3];
	uint8_t i;
	
	//printf_P(PSTR("X = %5d    "), acc_raw.s16[0] >> 4);
	//printf_P(PSTR("Y = %5d    "), acc_raw.s16[1] >> 4);
	//printf_P(PSTR("Z = %5d\n"),   acc_raw.s16[2] >> 4);
	
	// output value correction
	accPreIter[0] = ((float)acc_raw.s16[0] / unitG - OFFX) / SENSX;
	accPreIter[1] = ((float)acc_raw.s16[1] / unitG - OFFY) / SENSY;
	accPreIter[2] = ((float)acc_raw.s16[2] / unitG - OFFZ) / SENSZ;
	
	acc[0] = accPreIter[0];  acc[1] = accPreIter[1];  acc[2] = accPreIter[2];
	
	for(i = 0; i < iterOrder; i++) {
		acc[0] = accPreIter[0] - acc[1] * CXY - acc[2] * CXZ;
		acc[1] = accPreIter[1] - acc[2] * CYZ - acc[0] * CYX;
		acc[2] = accPreIter[2] - acc[0] * CZX - acc[1] * CZY;
	}
	
	values[0] = acc[0];  values[1] = acc[1];  values[2] = acc[2];
}

void read_acc_angles(float *angles) {
	float acc[3];
	read_acc_values(acc);
	
	angles[0] = atan(acc[0] / sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * 180.0 / M_PI;
	angles[1] = atan(acc[1] / sqrt(acc[2]*acc[2] + acc[0]*acc[0])) * 180.0 / M_PI;
	angles[2] = atan(acc[2] / sqrt(acc[0]*acc[0] + acc[1]*acc[1])) * 180.0 / M_PI;
}

