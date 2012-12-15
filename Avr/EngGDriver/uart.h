#include <avr/io.h>   
#include <avr/interrupt.h>   
#include <stdio.h>

void uart_init(void);		/* Initialize UART and Flush FIFOs */ 
uint8_t uart_get (void);	/* Get a byte from UART Rx FIFO */ 
uint8_t uart_available(void);	/* Check number of data in UART Rx FIFO */ 
int uart_put (char , FILE*);	/* Put a byte into UART Tx FIFO */ 
