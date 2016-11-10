/*
 * ATMega324_1.c
 *
 * Created: 25/04/2015 18:56:16
 *  Author: Mike
 */ 

#include "ukhasnet-rfm69.h"		// UKHasnet RFM69 library

//#define F_CPU 8000000UL						// Using the Internal RC Oscilator (8MHz) with CLKDIV/8 set)
#define BAUD_PC 19200							// define baud
#define BAUD_GPS 9600
#define BAUDRATE_PC ((F_CPU)/(BAUD_PC*16UL)-1)	// set baud rate value for UBRR
#define BAUDRATE_GPS ((F_CPU)/(BAUD_GPS*16UL)-1)	// set baud rate value for UBRR

#define SERIAL_TXBUF 80			// Size of Buffer for Serial Send
#define SERIAL_RXBUF 20			// Size of Buffer for Serial Receive
#define GPS_TXBUF 20			// Size of Buffer for GPS Send
#define GPS_RXBUF 80			// Size of Buffer for GPS Receive

#define BLINK_FREQ	1		// How often to Blink the LED (seconds)
#define TX_FREQ		5		// How often to transmit (seconds)
#define RX_DELAY	30		// Delay in main loop (ms)

#define HEX2INT(x) (x > '9')? (x &~ 0x20) - 'A' + 10: (x - '0')

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

const char latitude[]="50.94404";
const char longitude[]="-1.40552";
const char altitude[]="100";

char serial_tx[SERIAL_TXBUF+1];
uint8_t serial_tx_pos=0;

void serial_send(const char* tx){
	if (SERIAL_TXBUF < strlen(tx)){
		// Error
	} else {
 
		// Wait until the the buffer is free
		while (serial_tx_pos != 0)
			_delay_ms(1);

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
	char buff[2];
	memset(buff,0,2);
	PORTA ^= (1<<2);	// Toggle LED
	switch (data){
		default:
			buff[0]=data;
			serial_send(buff);
		break;
	}	
}

#ifdef _GPS
// GPS UART (rx)
ISR(USART1_RX_vect){
	uint8_t data = UDR1;
	char buff[2];
	memset(buff,0,2);
	buff[0]=data;
	serial_send(buff);

}
#endif

int main(void){
	memset(serial_tx,0,SERIAL_TXBUF+1);

	
	// Setup the Serial UART
	UBRR0H = (BAUDRATE_PC>>8);		// shift the register right by 8 bits
	UBRR0L = BAUDRATE_PC;			// set baud rate
	UCSR0B |= (1<<TXEN0); // | (1<<UDRIE0);	// Enable Transmit and Interupt	(Also TXCIE0)
	UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);	// Enable Recieve and Interrupt
	UCSR0C |= (1<<USBS0) | (3 << UCSZ00);	// 8 Data bit and 2 stop bit
	
#ifdef _GPS
	// Setup the GPS UART
	UBRR1H = (BAUDRATE_GPS>>8);		// shift the register right by 8 bits
	UBRR1L = BAUDRATE_GPS;			// set baud rate
	UCSR1B |= (1<<TXEN1); // | (1<<UDRIE0);	// Enable Transmit and Interupt	(Also TXCIE0)
	UCSR1B |= (1<<RXEN1) | (1<<RXCIE1);	// Enable Recieve and Interrupt
	UCSR1C |= (1<<USBS1) | (3 << UCSZ10);	// 8 Data bit and 2 stop bit
#endif


	// Setup LEDs
	// LEDs on Port A (set as output and turn on 1&3)
	DDRA |= (1<<0) | (1<<1) | (1<<2);
	PORTA |= (1<<0) | (1<<2);


	// Enable interupts
	sei();
	
	
	serial_send("Starting\r\n");

	// Setup SPI
	while(rf69_init() != RFM_OK)
		_delay_ms(100);
	serial_send("Spi Init\r\n");
	
	bool ispacket;
	int16_t lastrssi;
	rfm_reg_t len;
	char packet_buf[65], *p;

	char buff[SERIAL_TXBUF+1];

	int repeat=3;
	char sequence='a';

	uint16_t count_blink=0;
	uint16_t count_tx=0;

	// Main Loop
	while(1) {
		_delay_ms(RX_DELAY);

		// Recieve
		rf69_receive((rfm_reg_t *)packet_buf, &len, &lastrssi, &ispacket);

		/* The boolean variable 'ispacket' will be true if rf69_receive
		 * tells us there is a packet waiting in the RFM69 receive buffer
		 */
		if( ispacket ) {
			/* Find end of packet */
			p = memchr(packet_buf, ']', len);

			/* If it is a valid packet then send on serial */
			if (p != NULL) {
				*++p = '\0';
				sprintf(buff,"rx: %s|%d\r\n", packet_buf, lastrssi);
				serial_send(buff);
			}
		}

		if ( count_tx++ > (((uint16_t)TX_FREQ * 1000) / RX_DELAY)){
			// TODO, Build up the string a bit at a time, only add fields if they're good
			int8_t temp;
			rfm_status_t temp_status;
			temp_status = rf69_read_temp(&temp);

			if (('a' == sequence) || ('z' == sequence)){
				sprintf(buff,"tx: %d%cL%s,%s,%s[MA4]\r\n",repeat,sequence++,latitude, longitude, altitude);
			} else {
				sprintf(buff,"tx: %d%cT%d[MA4]\r\n",repeat,sequence++,temp);
			}
			
			serial_send(buff);
			//serial_send("tx: triggered\r\n");

			if (sequence>'z') sequence='b';
			count_tx=0;
		}

		if ( count_blink++ > (((uint16_t)BLINK_FREQ * 1000) / RX_DELAY)){
			PORTA ^= (1<<1);
			//serial_send(".");
			count_blink=0;
		}
	}
}
