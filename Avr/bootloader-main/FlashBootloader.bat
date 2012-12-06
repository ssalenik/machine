avrdude -c usbasp -p m1284p -F -U flash:w:ATmegaBOOT_1284P.hex:i -U hfuse:w:0b11011000:m -U lfuse:w:0b11100000:m -U lock:w:0b00101111:m
