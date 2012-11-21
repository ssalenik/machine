avrdude -c avrisp -P COM%1 -p atmega1284p -U flash:w:"%2.bin":r
