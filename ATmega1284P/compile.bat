del "%1.elf" "%1.lst" "%1.bin"
avr-gcc "%1.c" -o "%1.elf" -mmcu=atmega1284p -Wall -Os -Wl,-u,vfprintf -lprintf_flt -lm
avr-objcopy -O binary "%1.elf" "%1.bin"
avr-objdump -d "%1.elf" > "%1.lst"
