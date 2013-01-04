/** -----------------------------------------------------------------
 *      PORT CONNECTIONS
 *  -----------------------------------------------------------------
 *   #	NAME	ALT FUNCTIONS	I/O	P/U	DEFS	DEVICE
 *  -----------------------------------------------------------------
 *  40	PA0	ADC0		I	Y	Q3_Z	TURN_ZERO
 *  39	PA1	ADC1		I	N	SENS2	SENS2/RESERVED
 *  38	PA2	ADC2		O/C	N	SNDPLY	SND_PLAY
 *  37	PA3	ADC3		O	N	SNDRST	SND_RST
 *  36	PA4	ADC4		I	N	LIFT_FB	LIFT_FEEDBACK
 *  35	PA5	ADC5		I	N	VSENS	BATT_LEVEL
 *  34	PA6	ADC6		O	N	FET1	MOSFET1
 *  33	PA7	ADC7		O	N	FET2	MOSFET2
 *  -----------------------------------------------------------------
 *   1	PB0	T0		I	Y	BLD_SW	BOOTLOADER_SW
 *   2	PB1	T1		I	N	Q3_IO	TURN_QIO
 *   3	PB2	AIN0	INT2	I	N	Q3_INT	TURN_QINT
 *   4	PB3	AIN1	OC0A	O	N	PWM5	SERVO5
 *   5	PB4	SS	OC0B	O	N	PWM6	SERVO6
 *   6	PB5	MOSI	ICP3	O	N	SNDDAT	PROG/SND_DATA
 *   7	PB6	MISO	OC3A	I	Y	SNDBSY	PROG/SND_BUSY
 *   8	PB7	SCK	OC3B	O	N	SNDCLK	PROG/SND_CLK
 *  -----------------------------------------------------------------
 *  22	PC0	SCL		I	N		RESERVED
 *  23	PC1	SDA		I	N		RESERVED
 *  24  PC2			O	N	LED1	BATT_LED
 *  25  PC3			O	N	SPWR	SERVO_PWR
 *  26  PC4			O	N	DIR3A	TURN_DIR_A
 *  27  PC5			O	N	DIR3B	TURN_DIR_B
 *  28	PC6			O	N	DIR4A	LIFT_DIR_A
 *  29	PC7			O	N	DIR4B	LIFT_DIR_B
 *  -----------------------------------------------------------------
 *  14	PD0	RXD0	T3	I	Y	DRV_RX	COMM/PROG
 *  15	PD1	TXD0		O	N	DRV_TX	COMM/PROG
 *  16	PD2	RXD1	INT0	I	Y	DBG_RX	COMM
 *  17	PD3	TXD1	INT1	O	N	DBG_TX	COMM
 *  18	PD4	OC1B		O	N	PWM4	LIFT_PWM
 *  19	PD5	OC1A		O	N	PWM3	TURN_PWM
 *  20	PD6	OC2B	ICP1	O	N	PWM8	SERVO8
 *  21	PD7	OC2A		O	N	PWM7	SERVO7
 *  -----------------------------------------------------------------
 */

#define Q3_Z	A, 0
#define SENS2	   1
#define SNDPLY	A, 2
#define SNDRST	A, 3
#define LIFT_FB	   4
#define VSENS	   5
#define FET1	A, 6
#define FET2	A, 7

#define BLD_SW	B, 0
#define Q3_PIN	PINB
#define Q3_IO	   1
#define Q3_INT	   2
#define SNDDAT	B, 5
#define SNDBSY	B, 6
#define SNDCLK	B, 7

#define LED1	C, 2
#define SPWR	C, 3
#define DIR3A	C, 4
#define DIR3B	C, 5
#define DIR4A	C, 6
#define DIR4B	C, 7

#define set_bit(x) __set_b(x)
#define clr_bit(x) __clr_b(x)
#define set_ddr(x) __set_d(x)
#define clr_ddr(x) __clr_d(x)
#define bit_set(x) __b_set(x)
#define bit_clr(x) __b_clr(x)
#define __set_b(port, bit) ( _SFR_BYTE(PORT##port) |=  _BV(bit))
#define __clr_b(port, bit) ( _SFR_BYTE(PORT##port) &= ~_BV(bit))
#define __set_d(port, bit) ( _SFR_BYTE(DDR##port)  |=  _BV(bit))
#define __clr_d(port, bit) ( _SFR_BYTE(DDR##port)  &= ~_BV(bit))
#define __b_set(port, bit) ( _SFR_BYTE(PIN##port)  & _BV(bit))
#define __b_clr(port, bit) (~_SFR_BYTE(PIN##port)  & _BV(bit))

void init_ports(void) {
	DIDR0 = 0b00110010; // disable digital input buffers for analog inputs
	PORTA = 0b00000001; // for all PORTs see P/U column in the table above
	DDRA  = 0b11001000; // for all DDRs see I/O column in the table above
	PORTB = 0b11100001;
	DDRB  = 0b10111000;
	DDRC  = 0b11111100;
	PORTD = 0b00000101;
	DDRD  = 0b11111010;
}

void motor3_fwd(void) { clr_bit(DIR3B); set_bit(DIR3A); }
void motor3_rev(void) { clr_bit(DIR3A); set_bit(DIR3B); }
void motor4_fwd(void) { clr_bit(DIR4A); set_bit(DIR4B); }
void motor4_rev(void) { clr_bit(DIR4B); set_bit(DIR4A); }

