avrdude -c usbasp -p atmega1284p -F -U flash:w:"$1".bin:r -U hfuse:w:0b11011001:m -U lfuse:w:0b11100000:m
