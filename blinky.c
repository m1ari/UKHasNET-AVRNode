/*
 * ATMega324_1.c
 *
 * Created: 25/04/2015 18:56:16
 *  Author: Mike
 */ 

#define F_CPU 8000000UL						// Using the Internal RC Oscilator (8MHz) with CLKDIV/8 set)
#define BAUD_PC 19200							// define baud
#define BAUD_GPS 9600
#define BAUDRATE_PC ((F_CPU)/(BAUD_PC*16UL)-1)	// set baud rate value for UBRR
#define BAUDRATE_GPS ((F_CPU)/(BAUD_GPS*16UL)-1)	// set baud rate value for UBRR

#define SERIAL_TXBUF 20							// Size of Buffer for Serial Send
#define SERIAL_RXBUF 20							// Size of Buffer for Serial Receive
#define GPS_TXBUF 20							// Size of Buffer for GPS Send
#define GPS_RXBUF 80							// Size of Buffer for GPS Receive

#define HEX2INT(x) (x > '9')? (x &~ 0x20) - 'A' + 10: (x - '0')

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

char serial_tx[SERIAL_TXBUF+1];
uint8_t serial_tx_pos=0;

void serial_send(const char* tx){
	if (SERIAL_TXBUF < strlen(tx)){
		// Error
	} else {
 
		// Wait until the the buffer is free
		while (serial_tx_pos != 0);

		strncpy(serial_tx,tx,SERIAL_TXBUF);
		UCSR0B |= (1<<UDRIE0);		// Enable Transmit interrupt
	}
}
	

// Handle sending to the PC UART
ISR(USART0_UDRE_vect){
	// See if there's something to send
	if (serial_tx[serial_tx_pos] != '\0'){
		while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
		UDR0 = serial_tx[serial_tx_pos];
		serial_tx[serial_tx_pos]='\0';
		serial_tx_pos++;
	}

	//   If we're at the end               Or the next character is null
	if ((serial_tx_pos > SERIAL_TXBUF) | (serial_tx[serial_tx_pos] == '\0')) {
		serial_tx_pos=0;			// Reset to start of string
		UCSR0B &= ~(1<<UDRIE0);		// Clear Transmit interrupt
		
	}
}

// PC UART(rx)
ISR(USART0_RX_vect){
	char data = UDR0;
	PORTA ^= (1<<2);
	switch (data){
		default:
		break;
	}	
}

// GPS UART (rx)
ISR(USART1_RX_vect){
	static uint8_t state=0;			// State machine position.

	static uint8_t csum=0;			// Checksum calculation
	//static char nmea_csum[3];		// Storage for checksum (could we use buff)
	static uint8_t nmea_csum;		// Checksum from nmea string
	static uint8_t nmea_pos=0;		// Position for storing checksum (could use buff_pos)
	
	static char buff[20];			// Buffer for processing incomming data
	static uint8_t buff_pos=0;		// Position for writing the buffer

	static uint8_t nmea_hour;		// Time from NMEA
	static uint8_t nmea_min;
	static uint8_t nmea_sec;
	static char nmea_lat[11];		// Latitude from NMEA	+5056.6505
	static char nmea_lon[12];		// Longitue from NMEA	-00124.3531

	// Read in data byte
	uint8_t data = UDR1;

	// TODO buff overflow handling
	if (20 <= buff_pos) {
		memset(buff,0,20);
		buff_pos=0;
		state=0;
		serial_send("buff overflow\r\n");
	}

	if ((0 != state) && ('$' == data)) {
		state=0;
		serial_send("Reset State\r\n");
	}

	switch (state){
		case 0:	// Waiting for start of string
			if ('$' == data){
				// Clear out variables
				csum=0;
				//memset(nmea_csum,0,3);
				nmea_csum=0;
				nmea_pos=0;
				memset(buff,0,20);
				buff_pos=0;

				state=1;
			}
		break;
		case 1:	// Start of a string
			csum ^= data;
			if (',' == data){	// Increment state if 
				if (strcmp("GPGGA",buff) == 0){
					state=10;
					memset(buff,0,20);
					buff_pos=0;
					serial_send("GGA\r\n");
				} else {	// Unhandled strings
					state=9;
					memset(buff,0,20);
					buff_pos=0;
				}
				
			} else {
				buff[buff_pos++] = data;				
			}
		break;

		case 9:	// Unhandled sentences
			if ('*' == data){
				state=250;
			} else {
				csum ^= data;
			}
		break;

		// $GPGGA,212748.000,5056.6505,N,00124.3531,W,2,07,1.8,102.1,M,47.6,M,0.8,0000*6B
		//        Time      , Lat, N/S  , Long, E/W  , Lock, Sats, HDoP, Alt, Alt units, Geo Sep,
		// GPGGA
		case 10:	// GPGGA Time		212748.000
		case 11:	// GPGGA Latitude	5056.6505
		case 12:	// GPGGA N/S		N
		case 13:	// GPGGA Longitude	00124.3531
		case 14:	// GPGGA E/W		W
		case 15:	// GPGGA Lock		2
		case 16:	// GPGGA Sats		07
		case 17:	// GPGGA HDoP		1.8
		case 18:	// GPGGA Alt		102.1
		case 19:	// GPGGA Alt Units	M
		case 20:	// GPGGA ??		47.6
		case 21:	// GPGGA		M
		case 22:	// GPGGA		0.8
		case 23:	// GPGGA		0000*6B
			if ('*' == data) {
				state=250;
				// Update telem fields
				memset(buff,0,20);
				buff_pos=0;
			} else {
				csum ^= data;
				if (',' == data){
					// Do something based on state
					if (10 == state){			// Time
						//serial_send(buff);
						nmea_hour =  (buff[0]-'0') * 10;
						nmea_hour += (buff[1]-'0');
						nmea_min  =  (buff[2]-'0') * 10;
						nmea_min  += (buff[3]-'0');
						nmea_sec  =  (buff[4]-'0') * 10;
						nmea_sec  += (buff[5]-'0');
						snprintf(buff,20,"Time: %02u:%02u:%02u\r\n",nmea_hour,nmea_min,nmea_sec);
						serial_send(buff);
					} else if (11 == state){		// Latitude
						strncpy(&nmea_lat[1],buff,10);
					} else if (12 == state) {
						if (strcmp("S",buff)==0) {
							nmea_lat[0]='-';
						} else {
							nmea_lat[0]='+';
						}
					} else if (13 == state) {		// Longitude
						strncpy(&nmea_lon[1],buff,10);
					} else if (14 == state) {
						if (strcmp("W",buff)==0) {
							nmea_lon[0]='-';
						} else {
							nmea_lon[0]='+';
						}
						
					}
					// Increment state if 
					state++;
					memset(buff,0,20);
					buff_pos=0;
				} else {
					buff[buff_pos++] = data;				
				}
			}

		break;
		case 24:
			// Beyond the end of GPGGA
			state=0;
		break;
		case 250:	// End of string - Check Checksum
			if ( ('\r' == data) || ('\n' == data) ){
				// Test CSUM
				if (nmea_csum==csum) {
					serial_send(".");
				} else {
					serial_send("x");
				}
				//snprintf(buff,20,"G: %02X, %02X\r\n",nmea_csum,csum);
				//serial_send(buff);
				state=0;
			} else {
				//nmea_csum[nmea_pos++]=data;
				switch (nmea_pos++){
					case 0:
						nmea_csum  = HEX2INT(data) * 16;
						// (data - '0') * 10;
						//int v = (A > '9')? (A &~ 0x20) - 'A' + 10: (A - '0');
					break;
					case 1:
						nmea_csum += HEX2INT(data);
						//(data - '0') * 1;
					break;
					default:
						serial_send("Err: nmea_csum\r\n");
					break;
				}
			}
		break;
	}
	

}

int main(void){
	memset(serial_tx,0,SERIAL_TXBUF+1);

	
	// Setup the Serial UART
	UBRR0H = (BAUDRATE_PC>>8);			// shift the register right by 8 bits
	UBRR0L = BAUDRATE_PC;			// set baud rate
	UCSR0B |= (1<<TXEN0); // | (1<<UDRIE0);	// Enable Transmit and Interupt	(Also TXCIE0)
	UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);	// Enable Recieve and Interrupt
	UCSR0C |= (1<<USBS0) | (3 << UCSZ00);	// 8 Data bit and 2 stop bit
	
	// Setup the GPS UART
	UBRR1H = (BAUDRATE_GPS>>8);			// shift the register right by 8 bits
	UBRR1L = BAUDRATE_GPS;			// set baud rate
	UCSR1B |= (1<<TXEN1); // | (1<<UDRIE0);	// Enable Transmit and Interupt	(Also TXCIE0)
	UCSR1B |= (1<<RXEN1) | (1<<RXCIE1);	// Enable Recieve and Interrupt
	UCSR1C |= (1<<USBS1) | (3 << UCSZ10);	// 8 Data bit and 2 stop bit


	// Setup LEDs
	// LEDs on Port A (set as output and turn on 1&3)
	DDRA |= (1<<0) | (1<<1) | (1<<2);
	PORTA |= (1<<0) | (1<<2);


	// Enable interupts
	sei();
	
	
	serial_send("Starting");
	
    while(1)
    {
		while (1){
			PORTA ^= (1<<1);
			_delay_ms(1000);
		}
    }
}
