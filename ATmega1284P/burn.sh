avrdude -c bsd -P /dev/parport0 -p atmega1284p -U flash:w:"$1".bin:r -U hfuse:w:0b11011001:m -U lfuse:w:0b11100000:m
