/*
#define cmd_err() fprintf_P(&debug, (PGM_P)cmd_error)
char cmd_error[] PROGMEM = "Command error\r\n";
*/

void cmd_err(void) { fprintf_P(&debug, PSTR("Command error\r\n")); }

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
					
					case 'r': // toggle reverse passthrough from DRIVE MCU
					rev_passthru ^= 1;
					break;
					
					case 'd': // local dump on/off
					local_dump ^= 1;
					break;
					
					case 'o': // PID on/off
					pid_on ^= 1;
					if(pid_on) { fprintf_P(&debug, PSTR("PID on!\r\n")); reset_pid(); }
					else       { fprintf_P(&debug, PSTR("PID off!\r\n")); }
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
					
					case ' ':
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
					
					case 'a': // magnet
					if(inputptr != 2 || (inputbuf[1] & ~1) != '0') { cmd_err(); break; }
					if (inputbuf[1] == '0') {
						clr_bit(FET1);
						clr_bit(FET2);
					} else {
						set_bit(FET1);
						set_bit(FET2);
					}
					break;
					
					case '3': // turn motor commands
					if(!isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					switch(inputbuf[1]) {
						case '0': // forward (uint8_t speed)
						if(inputptr != 4) { cmd_err(); break; }
						motor3_fwd();
						set_speed_3(htoa(inputbuf[2], inputbuf[3]) * 40);
						break;
						
						case '1': // backwards (uint8_t speed)
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
						ENC3_P = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '5': // set I (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						ENC3_I = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '6': // set D (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						ENC3_D = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '7': // set noise gate (int16_t level)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						ENC3_NOISE_GATE = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						default:
						cmd_err();
					}
					break;
					
					case '4': // lift motor commands
					if(!isHex(inputbuf[2]) || !isHex(inputbuf[3])) { cmd_err(); break; }
					switch(inputbuf[1]) {
						case '0': // forward (uint8_t speed)
						if(inputptr != 4) { cmd_err(); break; }
						motor4_fwd();
						set_speed_4(htoa(inputbuf[2], inputbuf[3]) * 40);
						break;
						
						case '1': // backwards (uint8_t speed)
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
						ACTU_P = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '5': // set I (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						ACTU_I = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '6': // set D (int16_t factor)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						ACTU_D = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						case '7': // set noise gate (int16_t level)
						if(inputptr != 6 || !isHex(inputbuf[4]) || !isHex(inputbuf[5])) { cmd_err(); break; }
						ACTU_NOISE_GATE = htoa(inputbuf[2], inputbuf[3]) << 8 | htoa(inputbuf[4], inputbuf[5]);
						break;
						
						default:
						cmd_err();
					}
					break;
					
					case '5': // servo5 commands
					if(inputptr != 3 || !isHex(inputbuf[1]) || !isHex(inputbuf[2])) { cmd_err(); break; }
					servo5(htoa(inputbuf[1], inputbuf[2]));
					//OCR0A = htoa(inputbuf[1], inputbuf[2]);
					break;
					
					case '6': // servo6 commands
					if(inputptr != 3 || !isHex(inputbuf[1]) || !isHex(inputbuf[2])) { cmd_err(); break; }
					servo6(htoa(inputbuf[1], inputbuf[2]));
					//OCR0B = htoa(inputbuf[1], inputbuf[2]);
					break;
					
					case '7': // servo7 commands
					if(inputptr != 3 || !isHex(inputbuf[1]) || !isHex(inputbuf[2])) { cmd_err(); break; }
					servo7(htoa(inputbuf[1], inputbuf[2]));
					//OCR2A = htoa(inputbuf[1], inputbuf[2]);
					break;
					
					case '8': // servo8 commands
					if(inputptr != 3 || !isHex(inputbuf[1]) || !isHex(inputbuf[2])) { cmd_err(); break; }
					servo8(htoa(inputbuf[1], inputbuf[2]));
					//OCR2B = htoa(inputbuf[1], inputbuf[2]);
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

