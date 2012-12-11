void check_drive_uart(void) {
	static uint8_t inputbuf[RX_LINE_SIZE], inputptr = 0;
	uint8_t i, recv;
	
	while(uart_available(DRIVE)) {
		recv = uart_get(DRIVE);
		
		if(recv == '\r') {
			if(inputptr) {
				if(inputbuf[0] == '@') {
					// reverse command from MAIN MCU
					// TODO
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

