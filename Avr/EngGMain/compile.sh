# clean
rm -f "$1".elf "$1".lst "$1".bin

# compile
#avr-gcc "$1".c -o "$1".elf -mmcu=atmega1284p -Wall -Os
#avr-gcc "$1".c -o "$1".elf -mmcu=atmega1284p -Wall -Os -fsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
avr-gcc "$1".c -o "$1".elf -mmcu=atmega1284p -std=gnu99 -Wall -Os -Wl,-u,vfprintf -lprintf_flt -lm

# link
#avr-objcopy -O ihex "$1".elf "$1".hex
avr-objcopy -O binary "$1".elf "$1".bin

# disassemble
#avr-objdump -d -S "$1".elf > "$1".lst
avr-objdump -d "$1".elf > "$1".lst

# show program size and memory used
avr-size -C "$1".elf --mcu=atmega1284p
