/** -----------------------------------------------------------------
 *      PORT CONNECTIONS
 *  -----------------------------------------------------------------
 *   #	NAME	ALT	I/O	PU	DEFS	FUNCTION
 *  -----------------------------------------------------------------
 *  37	PA0	ADC0	I	N	DL_S	DIGITAL LEFT   SENS
 *  36	PA1	ADC1	I	N	DR_S	DIGITAL RIGHT  SENS
 *  35	PA2	ADC2	I	N	DA_S	DIGITAL AUX    SENS
 *  34	PA3	ADC3	I	N	DF_S	DIGITAL FRONT  SENS
 *  33	PA4	ADC4	I	N	M2_S	MOTOR 2        SENS
 *  32	PA5	ADC5	I	N	M1_S	MOTOR 1        SENS
 *  31	PA6	ADC6	I	N	AS_S	ANALOG SHORT   SENS
 *  30	PA7	ADC7	I	N	AM_S	ANALOG MIDDLE  SENS
 *  -----------------------------------------------------------------
 *  40	PB0		O	N	DL_C	DIGITAL LEFT   CTRL
 *  41	PB1		O	N	DR_C	DIGITAL RIGHT  CTRL
 *  42	PB2		O	N	DA_C	DIGITAL AUX    CTRL
 *  43	PB3		O	N	DF_C	DIGITAL FRONT  CTRL
 *  44	PB4	PCINT12	I	Y	BLD_SW	BOOTLOADER SWITCH
 *   1	PB5	MOSI	O	N	EM	FOR PROGRAMMING, EM
 *   2	PB6	MISO	I	N		FOR PROGRAMMING
 *   3	PB7	SCK	I	N		FOR PROGRAMMING
 *  -----------------------------------------------------------------
 *  19	PC0	SCL	O/C	Y	ACC_C	ACCELEROMETER SCL
 *  20	PC1	SDA	O/C	Y	ACC_D	ACCELEROMETER SDA
 *  25	PC6		O	N	AS_P	ANALOG SHORT  POWER
 *  26	PC7		O	N	AM_P	ANALOG MIDDLE POWER
 *  -----------------------------------------------------------------
 *   9	PD0	RXD0	I	Y	DRV_RX	SERIAL RX DRIVE MCU
 *  10	PD1	TXD0	O	N	DRV_TX	SERIAL TX DRIVE MCU
 *  11	PD2	RXD1	I	Y	DBG_RX	SERIAL RX DEBUG
 *  12	PD3	TXD1	O	N	DBG_TX	SERIAL TX DEBUG
 *  13	PD4	OC1B	O	N	M2_C	MOTOR 2        CTRL
 *  14	PD5	OC1A	O	N	M1_C	MOTOR 1        CTRL
 *  15	PD6	OC2B	O	N	AS_C	ANALOG SHORT   CTRL
 *  16	PD7	OC2A	O	N	AM_C	ANALOG MIDDLE  CTRL
 *  -----------------------------------------------------------------
 */

/** -------------------------------------------------------------------------
 *      SIGNAL PIN POSITIONS
 *  -------------------------------------------------------------------------
 *  ANALOG             MOTORS             DIGITAL            DIGITAL
 *  -------------------------------------------------------------------------
 *  PD6 PC6     PA6    PD4         PA4    PA2         PB2    PA0         PB0
 *  S_C S_P GND S_S    2_C 5V4 GND 2_S    A_S GND 5V0 A_C    L_S GND 5V0 L_C
 *  M_C M_P GND M_S    1_C 5V4 GND 1_S    F_S GND 5V0 F_C    R_S GND 5V0 R_C
 *  PD7 PC7     PA7    PD5         PA5    PA3         PB3    PA1         PB1
 *  -------------------------------------------------------------------------
 */

#define DL 0
#define DR 1
#define DA 2
#define DF 3
#define M2 4
#define M1 5
#define AS 6
#define AM 7

#define DL_S PORTA
#define DR_S PORTA
#define DA_S PORTA
#define DF_S PORTA
#define M2_S PORTA
#define M1_S PORTA
#define AS_S PORTA
#define AM_S PORTA

#define DL_C PORTB
#define DR_C PORTB
#define DA_C PORTB
#define DF_C PORTB

#define AS_P PORTC
#define AM_P PORTC

#define M2_C PORTD
#define M1_C PORTD
#define AS_C PORTD
#define AM_C PORTD

#define BLD_PORT PORTB
#define BLD_SW   4

#define ACC_PORT PORTC
#define ACC_C    0
#define ACC_D    1

#define DRV_PORT PORTD
#define DRV_RX   0
#define DRV_TX   1

#define DBG_PORT PORTD
#define DBG_RX   2
#define DBG_TX   3

#define EM_PORT  PORTB
#define EM       5

#define REG  0
#define DIR -1
#define PIN -2

#define PR(reg, port) (&port + reg)

inline void    set_port (volatile uint8_t *port, uint8_t data) { *port  =  data; }
inline void    set_bit  (volatile uint8_t *port, uint8_t bit)  { *port |=  _BV(bit); }
inline void    clr_bit  (volatile uint8_t *port, uint8_t bit)  { *port &= ~_BV(bit); }
inline uint8_t read_bit (volatile uint8_t *port, uint8_t bit)  { return (*port & _BV(bit)) ? 1 : 0; }

void init_ports(void) {
	// PORTA and DDRA do not require initialization (all inputs w/o pullups)
	DIDR0 = 1 << AS | 1 << AM; // disable digital input buffers for analog inputs
	PORTB = 1 << BLD_SW; // for debug
	DDRB  = 1 << DL | 1 << DR | 1 << DA | 1 << DF | 1 << EM;
	PORTC = 1 << ACC_C | 1 << ACC_D;
	DDRC  = 1 << AS | 1 << AM;
	PORTD = 1 << DRV_RX | 1 << DRV_TX | 1 << DBG_RX | 1 << DBG_TX;
	DDRD  = 1 << DRV_TX | 1 << DBG_TX | 1 << M2 | 1 << M1 | 1 << AS | 1 << AM;
}

inline uint8_t IR_hole_L(void) { return read_bit(PR(PIN, DL_S), DL); }
inline uint8_t IR_hole_R(void) { return read_bit(PR(PIN, DR_S), DR); }
inline uint8_t IR_hole_F(void) { return read_bit(PR(PIN, DF_S), DF); }

inline void IR_enable_L(void)  { set_bit(PR(REG, DL_C), DL); }
inline void IR_enable_R(void)  { set_bit(PR(REG, DR_C), DR); }
inline void IR_enable_F(void)  { set_bit(PR(REG, DF_C), DF); }

inline void IR_disable_L(void) { clr_bit(PR(REG, DL_C), DL); }
inline void IR_disable_R(void) { clr_bit(PR(REG, DR_C), DR); }
inline void IR_disable_F(void) { clr_bit(PR(REG, DF_C), DF); }

inline void IR_enable_dist(void)  { set_bit(PR(REG, AM_P), AM); }
inline void IR_disable_dist(void) { clr_bit(PR(REG, AM_P), AM); }

inline void enable_descend(void)  { set_bit(PR(REG, AS_P), AS); }
inline void disable_descend(void) { clr_bit(PR(REG, AS_P), AS); }

inline void EM_enable(void)    { set_bit(PR(REG, EM_PORT), EM); }
inline void EM_disable(void)   { clr_bit(PR(REG, EM_PORT), EM); }
inline uint8_t EM_status(void) { return read_bit(PR(REG, EM_PORT), EM); }

