avrdude -c bsd -P /dev/parport0 -p atmega168 -U flash:w:main.bin:r -U lfuse:w:0b10100010:m
