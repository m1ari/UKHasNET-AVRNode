/*
 * UKHASnet Repeater Node
 * Author: Mike Axford <mfaxford@gmail.com>
 * Ver 0.1 - 29/05/2016

 * Designed to run on a ATMega 324
 * USART0 - Connected to PC
 * USART1 - Connected to GPS
 * RFM69HW on SPI with ?? as SS
 * Green LED
 * Yellow LED
 * Red LED
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ukhasnet-rfm69.h"		// UKHasnet RFM69 library

#define _CONSOLE
#define _GPS

struct serial_t {
	char *tx_buff;
	char *rx_buff;
	uint8_t tx_pos;
	uint8_t rx_pos;
	bool rx_ready;
	uint8_t error;
};

#define ERR_TX_OVERFLOW 0x01
#define ERR_RX_OVERFLOW 0x02
#define ERR_RX_NOTREADY 0x04

#ifdef _CONSOLE
// Console Serial Port Setup
#define SERIAL_CONSOLE_BAUD 19200		// define baud
#define SERIAL_CONSOLE_BAUDRATE ((F_CPU)/(SERIAL_CONSOLE_BAUD*16UL)-1)	// set baud rate value for UBRR
#define SERIAL_CONSOLE_TXBUF 90			// Size of Buffer for Serial Send
#define SERIAL_CONSOLE_RXBUF 20			// Size of Buffer for Serial Receive
struct serial_t serial_console;
#endif

#ifdef _GPS
// GPS Serial Port Setup
#define SERIAL_GPS_BAUD 9600
#define SERIAL_GPS_BAUDRATE ((F_CPU)/(SERIAL_GPS_BAUD*16UL)-1)	// set baud rate value for UBRR
#define SERIAL_GPS_TXBUF 20			// Size of Buffer for GPS Send
#define SERIAL_GPS_RXBUF 80			// Size of Buffer for GPS Receive
struct serial_t serial_gps;
#endif

#define BLINK_FREQ	1		// How often to Blink the LED (seconds)
#define TX_FREQ		5		// How often to transmit (seconds)
#define RX_DELAY	1		// Delay in main loop (ms)

const char latitude[]="50.94404";
const char longitude[]="-1.40552";
const char altitude[]="100";


/*
struct location_t {
	char latitude[10];
	char logitude[10];
	char altitude[7];
	time_t fix_time;
};
*/

/* Send data to the Serial Console */
// TODO Can we make this generic so it can be used with both USARTs ?
void console_send(const char* tx){
	if (SERIAL_CONSOLE_TXBUF < strlen(tx)){
		serial_console.error |= ERR_TX_OVERFLOW;
		// Error
	} else {
 
		// Wait until the the buffer is free
		// TODO, this causes issues if console_send is called from an ISR
		// If we changed to a circular buffer we might not need to wait if there's space
		while (serial_console.tx_pos != 0)
			_delay_ms(1);

		strncpy(serial_console.tx_buff,tx,SERIAL_CONSOLE_TXBUF);
		UCSR0B |= (1<<UDRIE0);		// Enable Transmit interrupt
	}
}
	

// Interupt routine to handle sending data to the console
ISR(USART0_UDRE_vect){
	// See if there's something to send
	if (serial_console.tx_buff[serial_console.tx_pos] != '\0'){
		while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
		UDR0 = serial_console.tx_buff[serial_console.tx_pos];
		serial_console.tx_buff[serial_console.tx_pos]='\0';
		serial_console.tx_pos++;
	}

	//   If we're at the end               Or the next character is null
	if ((serial_console.tx_pos > SERIAL_CONSOLE_TXBUF) | (serial_console.tx_buff[serial_console.tx_pos] == '\0')) {
		serial_console.tx_pos=0;			// Reset to start of string
		UCSR0B &= ~(1<<UDRIE0);		// Clear Transmit interrupt
		
	}
}

// Reset the RX Buffer
void serial_rx_reset(struct serial_t *serial, uint8_t bufflen){
	// TODO It might be beneficial to store the buffer length in the struct
	serial->rx_ready=false;
	serial->rx_pos=0;
	memset(serial->rx_buff,0,bufflen+1);
}

// Interupt routine to handle receiving data from the console
ISR(USART0_RX_vect){
	char data = UDR0;

#ifdef _CONSOLE_ECHO
	// Echo output
	char buff[3];
	if ('\r' == data)
		sprintf(buff,"\r\n");
	else
		sprintf(buff,"%c",data);
	console_send(buff);		// TODO This could hang if there's other stuff in the serial buffer
#endif

	// TODO We ought to check rx_ready isn't set - We shouldn't touch the buffer if it's being processed
	// Update Buffer
	if ( ('\r' == data) || ('\n' == data) ) {		// Newline
		serial_console.rx_buff[serial_console.rx_pos++]='\0';
		serial_console.rx_ready=true;
	} else {
		serial_console.rx_buff[serial_console.rx_pos++]=data;
		if (SERIAL_CONSOLE_RXBUF == serial_console.rx_pos ){		// Buffer full
			//console_send("rx Buffer overflow\r\n");	// TODO More hang risk
			serial_console.error |= ERR_RX_OVERFLOW;
			serial_console.rx_ready=true;
		}
	}

	//serial_console.error |= ERR_RX_NOTREADY;
	//PORTA ^= (1<<2);	// Toggle LED
}

#ifdef _GPS
		//serial_gps.error |=ERR_TX_OVERFLOW;
// GPS UART (rx)
ISR(USART1_RX_vect){
	static char buff[SERIAL_GPS_RXBUF+1];
	static uint8_t pos=0;
	bool ready=false;

	char data = UDR1;

	// Update Buffer
	if ( ('\r' == data) || ('\n' == data) ) {		// Newline
		if (pos>0) {	// ignore if theres no string
			buff[pos++]='\0';
			ready=true;
		}
	} else {
		buff[pos++]=data;
		if (SERIAL_GPS_RXBUF == pos ){		// Buffer full
			//console_send("GPS rx Buffer overflow\r\n");	// TODO More hang risk
			buff[pos]='\0';
			serial_gps.error |=ERR_RX_OVERFLOW;
			ready=true;
		}
	}

	if (ready){
		if (!serial_gps.rx_ready){
			strcpy(serial_gps.rx_buff, buff);
			serial_gps.rx_ready=true;
		} else {
			serial_gps.error |=ERR_RX_NOTREADY;
			//console_send("GPS buffer not ready for update\r\n");	// TODO more Hang risk
		}
		pos=0;
	}
}
#endif

int main(void){
#ifdef _CONSOLE
	// TODO We could change malloc to alloca
	serial_console.tx_buff=malloc(sizeof(char) * (SERIAL_CONSOLE_TXBUF+1));
	serial_console.rx_buff=malloc(sizeof(char) * (SERIAL_CONSOLE_RXBUF+1));
	// TODO We should check the malloc works
	memset(serial_console.tx_buff,0,SERIAL_CONSOLE_TXBUF+1);
	serial_console.tx_pos = 0;
	serial_rx_reset(&serial_console, SERIAL_CONSOLE_RXBUF);
#endif

#ifdef _GPS
	// TODO We could change malloc to alloca
	serial_gps.tx_buff=malloc(sizeof(char) * (SERIAL_GPS_TXBUF+1));
	serial_gps.rx_buff=malloc(sizeof(char) * (SERIAL_GPS_RXBUF+1));

	// TODO We should check the malloc works
	memset(serial_gps.tx_buff,0,SERIAL_GPS_TXBUF+1);
	serial_gps.tx_pos = 0;
	serial_rx_reset(&serial_gps, SERIAL_GPS_RXBUF);
#endif

#ifdef _CONSOLE
	// Setup the Serial UART
	UBRR0H = (SERIAL_CONSOLE_BAUDRATE>>8);		// shift the register right by 8 bits
	UBRR0L = SERIAL_CONSOLE_BAUDRATE;			// set baud rate
	UCSR0B |= (1<<TXEN0); // | (1<<UDRIE0);	// Enable Transmit and Interupt	(Also TXCIE0)
	UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);	// Enable Recieve and Interrupt
	UCSR0C |= (1<<USBS0) | (3 << UCSZ00);	// 8 Data bit and 2 stop bit
#endif
	
#ifdef _GPS
	// Setup the GPS UART
	UBRR1H = (SERIAL_GPS_BAUDRATE>>8);		// shift the register right by 8 bits
	UBRR1L = SERIAL_GPS_BAUDRATE;			// set baud rate
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
	
	
	console_send("Starting\r\n");

	// Setup SPI
	while(rf69_init() != RFM_OK)
		_delay_ms(100);
	console_send("Spi Init\r\n");
	
	bool ispacket;
	int16_t lastrssi;
	rfm_reg_t len;
	char packet_buf[65], *p;

	char buff[SERIAL_CONSOLE_TXBUF+1];

	int repeat=3;
	char sequence='a';

	uint16_t count_blink=0;
	uint16_t count_tx=0;

	// Main Loop
	while(1) {
		if (serial_gps.rx_ready){
			snprintf(buff,SERIAL_CONSOLE_TXBUF+1,"GPS: %s\r\n",serial_gps.rx_buff);
			console_send(buff);
			serial_rx_reset(&serial_gps, SERIAL_GPS_RXBUF);
			buff[0]='\0';
		}

		if (serial_console.error > 0){
			snprintf(buff,SERIAL_CONSOLE_TXBUF+1,"Console Error: %#.4x\r\n",serial_console.error);

			if (serial_console.error & ERR_TX_OVERFLOW )
				strncat(buff,"\ttx overflow\r\n",SERIAL_CONSOLE_TXBUF-strlen(buff));
			if (serial_console.error & ERR_RX_OVERFLOW )
				strncat(buff,"\trx overflow\r\n",SERIAL_CONSOLE_TXBUF-strlen(buff));
			if (serial_console.error & ERR_RX_NOTREADY )
				strncat(buff,"\trx not ready\r\n",SERIAL_CONSOLE_TXBUF-strlen(buff));

			serial_console.error=0;
			console_send(buff);
			buff[0] = '\0';
		}

		if (serial_gps.error > 0){
			snprintf(buff,SERIAL_CONSOLE_TXBUF+1,"GPS Error: %#.4x\r\n",serial_gps.error);

			if (serial_gps.error & ERR_TX_OVERFLOW )
				strncat(buff,"\ttx overflow\r\n",SERIAL_CONSOLE_TXBUF-strlen(buff));
			if (serial_gps.error & ERR_RX_OVERFLOW )
				strncat(buff,"\trx overflow\r\n",SERIAL_CONSOLE_TXBUF-strlen(buff));
			if (serial_gps.error & ERR_RX_NOTREADY )
				strncat(buff,"\trx not ready\r\n",SERIAL_CONSOLE_TXBUF-strlen(buff));

			serial_gps.error=0;
			console_send(buff);
			buff[0] = '\0';
		}

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
				console_send(buff);
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
			
			console_send(buff);
			//console_send("tx: triggered\r\n");

			if (sequence>'z') sequence='b';
			count_tx=0;
		}

		if ( count_blink++ > (((uint16_t)BLINK_FREQ * 1000) / RX_DELAY)){
			PORTA ^= (1<<1);
			//console_send(".");
			count_blink=0;
		}
	}
}

//#define HEX2INT(x) (x > '9')? (x &~ 0x20) - 'A' + 10: (x - '0')
