# clean
rm -f main.elf main.lst main.bin
# compile
avr-gcc -Wall -Os -mmcu=atmega168 -o main.elf main.c
# link
avr-objcopy -O binary main.elf main.bin
# disassemble
avr-objdump -d main.elf > main.lst
