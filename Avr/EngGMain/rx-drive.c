void check_drive_uart(void) {
	static uint8_t inputbuf[RX_LINE_SIZE], inputptr = 0;
	uint8_t i, recv;
	
	while(uart_available(DRIVE)) {
		recv = uart_get(DRIVE);
		
		if(recv == '\r') {
			if(inputptr) {
				if(inputbuf[0] == '@') {
					// reverse command from DRIVE MCU
					switch(inputbuf[1]) {
						case '1': // drive complete
						if(inputptr != 3 || inputbuf[2] != '0') { break; }
						drive_complete = 1;
						break;
						
						case '4': // track info update
						switch(inputbuf[2]) {
							case '4': // absolute position
							if(inputptr != 11) { break; }
							abspL = htoa(inputbuf[ 3], inputbuf[ 4]) << 8 | htoa(inputbuf[ 5], inputbuf[ 6]);
							abspR = htoa(inputbuf[ 7], inputbuf[ 8]) << 8 | htoa(inputbuf[ 9], inputbuf[10]);
							break;
							
							case '5': // relative position
							if(inputptr != 15) { break; }
							sectL = htoa(inputbuf[ 3], inputbuf[ 4]);
							relpL = htoa(inputbuf[ 5], inputbuf[ 6]) << 8 | htoa(inputbuf[ 7], inputbuf[ 8]);
							sectL = htoa(inputbuf[ 9], inputbuf[10]);
							relpL = htoa(inputbuf[11], inputbuf[12]) << 8 | htoa(inputbuf[13], inputbuf[14]);
							break;
							
							case '6': // track sensors
							if(inputptr != 7) { break; }
							trksL = htoa(inputbuf[3], inputbuf[4]);
							trksR = htoa(inputbuf[5], inputbuf[6]);
							break;
						}
						break;
					}
				}
				else if(rev_passthru) {
					for(i = 0; i < inputptr; i++) { uart_put(DEBUG, inputbuf[i]); }
					fprintf(&debug, "\r\n");
				}
				
				inputptr = 0;
			}
		}
		else if(recv == '\n') {
			;
		}
		else {
			if(inputptr != RX_LINE_SIZE) {
				inputbuf[inputptr] = recv;
				inputptr++;
			}
		}
	}
}

