void check_debug_uart(void) {
	static uint8_t inputbuf[RX_LINE_SIZE], inputptr = 0;
	uint8_t i, recv;
	
	while(uart_available(DEBUG)) {
		recv = uart_get(DEBUG);
		
		if(recv == '\r') {
			fprintf(&debug, "\r\n");
			
			if(inputptr) {
				switch(inputbuf[0]) {
					case '?': // print drive command list
					fprintf_P(&debug, PSTR("stop() ......................... | p30\r\n"));
					fprintf_P(&debug, PSTR("forward(speed) ................. | p31%%02x00\r\n"));
					fprintf_P(&debug, PSTR("backwards(speed) ............... | p31%%02x11\r\n"));
					fprintf_P(&debug, PSTR("forward_dist(speed, dist) ...... | p32%%02x%%04x\r\n"));
					fprintf_P(&debug, PSTR("backwards_dist(speed, dist) .... | p33%%02x%%04x\r\n"));
					fprintf_P(&debug, PSTR("freedrive(speedL, speedR, dir) . | p34%%02x%%02x%%02x\r\n"));
					break;
					
					case 'p': // passthrough to DRIVE MCU
					for(i = 1; i < inputptr; i++) { uart_put(DRIVE, inputbuf[i]); }
					uart_put(DRIVE, '\r');
					break;
					
					case '0': // reverse passthrough OFF
					rev_passthru = 0;
					break;
					
					case '1': // reverse passthrough ON
					rev_passthru = 1;
					break;
					
					case 's': // start/stop main thread
					run_main ^= 1;
					if(run_main) { fprintf_P(&debug, PSTR("Main thread started!\r\n")); }
					else         { fprintf_P(&debug, PSTR("Main thread stopped!\r\n")); stop(); }
					break;
					
					case 't': // start/stop test thread
					if(inputptr == 2) {
						run_test = htoa(0, inputbuf[1]);
						if(!run_test) { fprintf_P(&debug, PSTR("All test sequences stopped!\r\n")); stop(); }
						else { fprintf_P(&debug, PSTR("Started test sequence %u!\r\n"), run_test); }
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case ' ':
					stop();
					set_speed_3(0);
					set_speed_4(0);
					break;
					
					case 'u':
					fprintf_P(&debug, PSTR("%lu\r\n"), uptime());
					break;
					
					case 'b':
					#define VBAT_FACTOR 0.0044336
					fprintf_P(&debug, PSTR("4 x %1.2fV\r\n"), (float)read_adc(VSENS) * VBAT_FACTOR);
					break;
					
					case 'm': // servo power
					if(inputptr >= 2 && (inputbuf[1] & ~1) == '0') {
						inputbuf[1] == '0' ? clr_bit(SPWR) : set_bit(SPWR);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '3': // turn motor commands
					if(inputptr >= 4 && (inputbuf[1] & ~1) == '0' && isHex(inputbuf[2]) && isHex(inputbuf[3])) {
						if(inputbuf[1] == '0') { clr_bit(DIR3A); set_bit(DIR3B); }
						else                   { clr_bit(DIR3B); set_bit(DIR3A); }
						set_speed_3(htoa(inputbuf[2], inputbuf[3]) * 40);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '4': // turn motor commands
					if(inputptr >= 4 && (inputbuf[1] & ~1) == '0' && isHex(inputbuf[2]) && isHex(inputbuf[3])) {
						if(inputbuf[1] == '0') { clr_bit(DIR4A); set_bit(DIR4B); }
						else                   { clr_bit(DIR4B); set_bit(DIR4A); }
						set_speed_4(htoa(inputbuf[2], inputbuf[3]) * 40);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '5': // servo5 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo5(htoa(inputbuf[1], inputbuf[2]));
						//OCR0A = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '6': // servo6 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo6(htoa(inputbuf[1], inputbuf[2]));
						//OCR0B = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '7': // servo7 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo7(htoa(inputbuf[1], inputbuf[2]));
						//OCR2A = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					case '8': // servo8 commands
					if(inputptr >= 3 && isHex(inputbuf[1]) && isHex(inputbuf[2])) {
						servo8(htoa(inputbuf[1], inputbuf[2]));
						//OCR2B = htoa(inputbuf[1], inputbuf[2]);
					}
					else { fprintf_P(&debug, PSTR("Command error\r\n")); }
					break;
					
					default:
					fprintf_P(&debug, PSTR("Command error\r\n"));
				}
				
				inputptr = 0;
			}
		}
		else if(recv == 0x7f) {
			if(!inputptr) { uart_put(DEBUG, '\a'); }
			else {
				fprintf(&debug, "\b\e[K");
				inputptr--;
			}
		}
		else {
			if(inputptr == RX_LINE_SIZE) { uart_put(DEBUG, '\a'); }
			else {
				uart_put(DEBUG, recv);
				inputbuf[inputptr] = recv;
				inputptr++;
			}
		}
	}
}

