/*
#define cmd_err() fprintf_P(&debug, (PGM_P)cmd_error)
char cmd_error[] PROGMEM = "Command error\r\n";
*/

void cmd_err(void) { fprintf_P(&debug, PSTR("Command error\r\n")); }

void check_debug_uart(void) {
	static uint8_t inputbuf[RX_LINE_SIZE], inputptr = 0;
	uint8_t i, recv;
	int16_t ticks;
	
	while(uart_available(DEBUG)) {
		recv = uart_get(DEBUG);
		
		if(recv == '\r') {
			fprintf(&debug, "\r\n");
			
			if(inputptr) {
				switch(inputbuf[0]) {
					case '?': // print drive command list
					fprintf_P(&debug, PSTR("stop() ........................ | p00\r\n"));
					fprintf_P(&debug, PSTR("fwd_both(speed) ............... | p0300, p0400, p15 u8\r\n"));
					fprintf_P(&debug, PSTR("rev_both(speed) ............... | p0301, p0401, p15 u8\r\n"));
					fprintf_P(&debug, PSTR("forward(Lspeed, Rspeed) ....... | p0300, p0400, p11 u8, p12 u8\r\n"));
					fprintf_P(&debug, PSTR("reverse(Lspeed, Rspeed) ....... | p0301, p0401, p11 u8, p12 u8\r\n"));
					fprintf_P(&debug, PSTR("turnCCW(Lspeed, Rspeed) ....... | p0301, p0400, p11 u8, p12 u8\r\n"));
					fprintf_P(&debug, PSTR("turnCW (Lspeed, Rspeed) ....... | p0300, p0401, p11 u8, p12 u8\r\n"));
					fprintf_P(&debug, PSTR("set_abs_pos(pos) .............. | p1a s16\r\n"));
					fprintf_P(&debug, PSTR("set_rel_pos(sect, pos) ........ | p1b u8 u8\r\n"));
					fprintf_P(&debug, PSTR("pos_corr_on() ................. | p1f01\r\n"));
					fprintf_P(&debug, PSTR("pos_corr_off() ................ | p1f00\r\n"));
					fprintf_P(&debug, PSTR("nav_abs_pos(speed, pos) ....... | p31 u8 s16\r\n"));
					fprintf_P(&debug, PSTR("nav_rel_pos(speed, sect, pos) . | p32 u8 u8 u8\r\n"));
					break;
					
					case 'p': // passthrough to DRIVE MCU
					for(i = 1; i < inputptr; i++) { uart_put(DRIVE, inputbuf[i]); }
					uart_put(DRIVE, '\r');
					break;
					
					case 'r': // toggle reverse passthrough from DRIVE MCU
					rev_passthru ^= 1;
					break;
					
					case 'd': // local dump on/off
					local_dump ^= 1;
					break;
					
					case 's': // start/stop main thread
					run_main ^= 1;
					if(run_main) { fprintf_P(&debug, PSTR("Main thread started!\r\n")); }
					else         { fprintf_P(&debug, PSTR("Main thread stopped!\r\n")); stop(); }
					break;
					
					case 't': // start/stop test thread
					if(inputptr != 2) { cmd_err(); break; }
					run_test = htoa(0, inputbuf[1]);
					if(!run_test) { fprintf_P(&debug, PSTR("All test sequences stopped!\r\n")); stop(); }
					else { fprintf_P(&debug, PSTR("Started test sequence %u!\r\n"), run_test); }
					break;
					
					case ' ': // stop all motors
					run_main = 0;
					run_test = 0;
					pid_on = 0;
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
					if(inputptr != 2 || (inputbuf[1] & ~1) != '0') { cmd_err(); break; }
					inputbuf[1] == '0' ? clr_bit(SPWR) : set_bit(SPWR);
					break;
					
					case '9': // magnets
					if(inputptr != 2 || (inputbuf[1] & ~1) != '0') { cmd_err(); break; }
					if(inputbuf[1] == '0') { clr_bit(FET1); clr_bit(FET2); fprintf_P(&debug, PSTR("Magnets off!\r\n")); }
					else                   { set_bit(FET1); set_bit(FET2); fprintf_P(&debug, PSTR("Magnets on!\r\n"));  }
					break;
					
					case '1': // PID on/off
					if(inputptr != 2 || (inputbuf[1] & ~1) != '0') { cmd_err(); break; }
					if(inputbuf[1] == '0') { pid_on = 0; fprintf_P(&debug, PSTR("PID off!\r\n")); }
					else                   { pid_on = 1; fprintf_P(&debug, PSTR("PID on!\r\n")); reset_pid(); }
					break;
					
					case '3': // turn motor commands
					if(!isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					switch(inputbuf[1]) {
						case '0': // forward (uint8_t speed)
						if(inputptr != 4) { cmd_err(); break; }
						motor3_fwd();
						set_speed_3(htoa(inputbuf[2], inputbuf[3]) * 40);
						break;
						
						case '1': // reverse (uint8_t speed)
						if(inputptr != 4) { cmd_err(); break; }
						motor3_rev();
						set_speed_3(htoa(inputbuf[2], inputbuf[3]) * 40);
						break;
						
						case '2': // set reference target (int16_t ticks)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						pid_target[MOTOR3] = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '3': // set reference speed (int16_t ticks)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						pid_speed[MOTOR3] = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '4': // set P (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ENC3_P = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '5': // set I (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ENC3_I = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '6': // set D (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ENC3_D = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '7': // set noise gate (int16_t level)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ENC3_NOISE_GATE = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case 'f': // set reference angle
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						ticks = deg2ticks(htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]));
						cli();
						V_encoder = ticks;
						sei();
						reset_pid();
						break;
						
						default:
						cmd_err();
					}
					break;
					
					case '4': // lift motor commands
					if(!isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					switch(inputbuf[1]) {
						case '0': // up (uint8_t speed)
						if(inputptr != 4) { cmd_err(); break; }
						motor4_fwd();
						set_speed_4(htoa(inputbuf[2], inputbuf[3]) * 40);
						break;
						
						case '1': // down (uint8_t speed)
						if(inputptr != 4) { cmd_err(); break; }
						motor4_rev();
						set_speed_4(htoa(inputbuf[2], inputbuf[3]) * 40);
						break;
						
						case '2': // set reference target (int16_t ticks)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						pid_target[MOTOR4] = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '3': // set reference speed (int16_t ticks)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						pid_speed[MOTOR4] = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '4': // set P (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ACTU_P = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '5': // set I (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ACTU_I = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '6': // set D (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ACTU_D = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '7': // set noise gate (int16_t level)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						//ACTU_NOISE_GATE = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						default:
						cmd_err();
					}
					break;
					
					case '5': // servo5 commands
					if(inputptr != 4 || !isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					servo5(htoa(inputbuf[2], inputbuf[3]));
					//OCR0A = htoa(inputbuf[1], inputbuf[2]);
					break;
					
					case '6': // servo6 commands
					if(inputptr != 4 || !isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					servo6(htoa(inputbuf[2], inputbuf[3]));
					//OCR0B = htoa(inputbuf[1], inputbuf[2]);
					break;
					
					case '7': // servo7 commands
					if(inputptr != 4 || !isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					servo7(htoa(inputbuf[2], inputbuf[3]));
					//OCR2A = htoa(inputbuf[1], inputbuf[2]);
					break;
					
					case '8': // servo8 commands
					if(inputptr != 4 || !isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					servo8(htoa(inputbuf[2], inputbuf[3]));
					//OCR2B = htoa(inputbuf[1], inputbuf[2]);
					break;
					
					case 'x':
					if(!isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					switch(inputbuf[1]) {					
						case '1': // set reference angle
							if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
							param1 = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
							break;
						case '2': // set reference angle
							if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
							param2 = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
							break;
						case '3': // set reference angle
							if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
							param3 = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
							break;
						case '4': // set reference angle
							if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
							param4 = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
							break;
						case '5': // set reference angle
							if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
							param5 = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
							break;
					}
					break;
					
					default:
					cmd_err();
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

