#define BAUD 115200
#define FIFO_LENGTH 128
// NOTE: FIFO_LENGTH must be an integer power of 2

#define DRIVE 0
#define DEBUG 1

// USART0 is used for communication with driving CPU
// USART1 is used for debugging

typedef struct {
	volatile uint8_t buff[FIFO_LENGTH];
	volatile uint8_t head, tail, size;
} FIFO;

FIFO tx0, rx0, tx1, rx1;

void init_fifo(FIFO *fifo_struct) {
	fifo_struct->head = 0;
	fifo_struct->tail = 0;
	fifo_struct->size = 0;
}

void init_uart(void) {
	uint16_t prescaler = (F_CPU / BAUD - 4) / 8;
	
	init_fifo(&tx0);
	init_fifo(&rx0);
	init_fifo(&tx1);
	init_fifo(&rx1);
	
	UBRR0  = prescaler;
	UCSR0A = 1 << U2X0; // set oversampling FSM to 8 instead of 16
	UCSR0B = 1 << RXCIE0 | 1 << RXEN0 | 1 << TXEN0;
	
	UBRR1  = prescaler;
	UCSR1A = 1 << U2X1; // set oversampling FSM to 8 instead of 16
	UCSR1B = 1 << RXCIE1 | 1 << RXEN1 | 1 << TXEN1;
}

uint8_t uart_available(uint8_t port) { return port ? rx1.size : rx0.size; }

uint8_t uart_get(uint8_t port) {
	uint8_t c, ptr;
	FIFO *rx;
	rx = port ? &rx1 : &rx0;
	
	while(!rx->size); // wait until a byte is received
	
	ptr = rx->head;
	c = rx->buff[ptr];
	ptr++;
	ptr &= (FIFO_LENGTH - 1); // circular buffer
	rx->head = ptr;
	
	cli();
	rx->size--;
	sei();
	
	return c;
}

void uart_put(uint8_t port, uint8_t c) {
	uint8_t ptr;
	FIFO *tx;
	tx = port ? &tx1 : &tx0;
	
	while(tx->size == FIFO_LENGTH); // wait if buffer is full
	
	ptr = tx->tail;
	tx->buff[ptr] = c;
	ptr++;
	ptr &= (FIFO_LENGTH - 1); // circular buffer
	tx->tail = ptr;
	
	cli();
	tx->size++;
	if(port) { UCSR1B = 1 << RXCIE1 | 1 << UDRIE1 | 1 << RXEN1 | 1 << TXEN1; }
	else     { UCSR0B = 1 << RXCIE0 | 1 << UDRIE0 | 1 << RXEN0 | 1 << TXEN0; }
	sei();
}

int drive_uart_put(char c, FILE *stream) { uart_put(0, c); return 0; }
int debug_uart_put(char c, FILE *stream) { uart_put(1, c); return 0; }

static FILE drive = FDEV_SETUP_STREAM(drive_uart_put, NULL, _FDEV_SETUP_WRITE);
static FILE debug = FDEV_SETUP_STREAM(debug_uart_put, NULL, _FDEV_SETUP_WRITE);

void RX_vect(uint8_t port) {
	uint8_t c, ptr;
	FIFO *rx;
	if(port) { c = UDR1; rx = &rx1; }
	else     { c = UDR0; rx = &rx0; }
	
	if(rx->size == FIFO_LENGTH) return; // exit to prevent buffer overflow
	
	ptr = rx->tail;
	rx->buff[ptr] = c;
	ptr++;
	ptr &= (FIFO_LENGTH - 1); // circular buffer
	rx->tail = ptr;
	
	rx->size++;
}

void UDRE_vect(uint8_t port) {
	uint8_t n, c, ptr;
	FIFO *tx;
	tx = port ? &tx1 : &tx0;
	
	ptr = tx->head;
	c = tx->buff[ptr];
	ptr++;
	ptr &= (FIFO_LENGTH - 1); // circular buffer
	tx->head = ptr;
	
	n = tx->size;
	n--;
	tx->size = n;
	
	if(port) {
		UDR1 = c;
		if(!n) { UCSR1B = 1 << RXCIE1 | 1 << RXEN1 | 1 << TXEN1; }
	}
	else {
		UDR0 = c;
		if(!n) { UCSR0B = 1 << RXCIE0 | 1 << RXEN0 | 1 << TXEN0; }
	}
}

SIGNAL(USART0_RX_vect) { RX_vect(0); }
SIGNAL(USART1_RX_vect) { RX_vect(1); }

SIGNAL(USART0_UDRE_vect) { UDRE_vect(0); }
SIGNAL(USART1_UDRE_vect) { UDRE_vect(1); }

