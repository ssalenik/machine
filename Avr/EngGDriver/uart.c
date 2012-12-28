/*------------------------------------------------*/   
/* UART functions                                 */   
#include "uart.h"     
   
#define BAUD  115200
   
typedef struct _fifo {   
    uint8_t idx_w;   
    uint8_t idx_r;   
    uint8_t count;   
    uint8_t buff[128];   
} FIFO;   
   
   
static volatile FIFO txfifo, rxfifo;   
   
   
   
/* Initialize UART */   
   
void uart_init()   
{   
    rxfifo.idx_r = 0;   
    rxfifo.idx_w = 0;   
    rxfifo.count = 0;   
    txfifo.idx_r = 0;   
    txfifo.idx_w = 0;   
    txfifo.count = 0;   
   
	  // Calculate the baud rate setting
		// the + 4 is there for rounding purposes
    UBRR0L = (F_CPU/BAUD + 4)/8 - 1;
		UCSR0A = _BV(U2X0); // set prescaler to 8 instead of 16
    UCSR0B = _BV(RXEN0)|_BV(RXCIE0)|_BV(TXEN0);   
}   
   
   
/* Get a received character */   
   
uint8_t uart_available ()   
{   
    return rxfifo.count;   
}   
   
   
uint8_t uart_get ()   
{   
    uint8_t d, i;   
   
   
    i = rxfifo.idx_r;   
    while(rxfifo.count == 0);   
    d = rxfifo.buff[i++];   
    cli();   
    rxfifo.count--;   
    sei();   
    if(i >= sizeof(rxfifo.buff))   
        i = 0;   
    rxfifo.idx_r = i;   
   
    return d;   
}   
   
  
/* Put a character to transmit */   
   
int uart_put (char d, FILE *stream)
{   
    uint8_t i;   
   
   
    i = txfifo.idx_w;   
    while(txfifo.count >= sizeof(txfifo.buff));   
    txfifo.buff[i++] = d;   
    cli();   
    txfifo.count++;   
    UCSR0B = _BV(RXEN0)|_BV(RXCIE0)|_BV(TXEN0)|_BV(UDRIE0);   
    sei();   
    if(i >= sizeof(txfifo.buff))   
        i = 0;   
    txfifo.idx_w = i;
		return 0;
}   
   
   
/* UART RXC interrupt */   
   
ISR(USART_RX_vect)   
{   
    uint8_t d, n, i;   
   
   
    d = UDR0;   
    n = rxfifo.count;   
    if(n < sizeof(rxfifo.buff)) {   
        rxfifo.count = ++n;   
        i = rxfifo.idx_w;   
        rxfifo.buff[i++] = d;   
        if(i >= sizeof(rxfifo.buff))   
            i = 0;   
        rxfifo.idx_w = i;   
    }   
}   
   
   
/* UART UDRE interrupt */   

ISR(USART_UDRE_vect)   
{   
    uint8_t n, i;   
   
   
    n = txfifo.count;   
    if(n) {   
        txfifo.count = --n;   
        i = txfifo.idx_r;   
        UDR0 = txfifo.buff[i++];   
        if(i >= sizeof(txfifo.buff))   
            i = 0;   
        txfifo.idx_r = i;   
    }   
    if(n == 0)   
        UCSR0B = _BV(RXEN0)|_BV(RXCIE0)|_BV(TXEN0);   
}   
