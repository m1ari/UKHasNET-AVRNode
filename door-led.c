#define F_CPU 8000000UL				// Using the Internal RC Oscilator (8MHz) with CLKDIV/8 set)
#define BAUD 9600				// define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)	// set baud rate value for UBRR

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

char count='A';

#define SERIAL_TXSIZE 10	// Characters we can store
char serial_txbuffer[SERIAL_TXSIZE+1];
uint8_t serial_txpos=0;


ISR(SPI_STC_vect) {
	// Read SPDR
	//while (!( UCSR0A & (1<<UDRE0)));	// Wait for the uart to be free
	//UDR0=SPDR;				// send the value we got on spi

	// If waiting for command decide what todo

	// Set SPDR
	SPDR=count++;
	if (count>'Z')
		count='A';
}

ISR(USART_RX_vect){
	char data=UDR0;
	PORTD ^= (1<<5);		// Toggle LEDs
}

ISR(USART_UDRE_vect){
	// See if there's something to send
	if (serial_txbuffer[serial_txpos] != '\0'){
		while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
		UDR0 = serial_txbuffer[serial_txpos];          
		serial_txbuffer[serial_txpos]='\0';
		serial_txpos++;
	}

	//   If we're at the end               Or the next character is null
	if ((serial_txpos > SERIAL_TXSIZE) | (serial_txbuffer[serial_txpos] == '\0')) {
		serial_txpos=0;			// Reset to start of string
		UCSR0B &= ~(1<<UDRIE0);		// Clear Transmit interrupt
		
	}
}

void sys_init() {
	memset(serial_txbuffer,0,SERIAL_TXSIZE+1);
	serial_txpos=0;
	
}

void port_init() {
	DDRD |= (1<<6) | (1<<5);	// Set LED as Output
	PORTD |= (1<<5);		// Turn LED1 On
	PORTD |= (1<<6);		// Turn LED2 on
	//PORTD &= ~(1<<6);		// Turn LED2 off

	DDRB |= (1<<4);			//MISO as OUTPUT// Set MISO as output
	SPCR = (1<<SPE) | (1<< SPIE);	//Enable SPI and SPI Interrupts

	//Uart 8n1 9600
  
	UBRR0H = (BAUDRATE>>8);			// shift the register right by 8 bits
	UBRR0L = BAUDRATE;			// set baud rate
	UCSR0B |= (1<<TXEN0); // | (1<<UDRIE0);	// Enable Transmit and Interupt	(Also TXCIE0)
	UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);	// Enable Recieve and Interrupt
	UCSR0C |= (1<<USBS0) | (3 << UCSZ00);	// 8 Data bit and 2 stop bit
}

int main(void) {	
	port_init();	// Configure Ports
	sys_init();	// Configure Variables

	sei();		// Enable interrupts

	// TODO need a way to determine if serial_buffer can be written to
	while ( (serial_txpos!=0) || serial_txbuffer[0]!= '\0');	// Wait whilst previous buffer is sent
	strncpy(serial_txbuffer, "Start\r\n",SERIAL_TXSIZE);
	UCSR0B |= (1<<UDRIE0);		// Enable Transmit interrupt

	while (1) {
		//PORTD ^= (1<<6) | (1<<5);		// Toggle LEDs
		PORTD ^= (1<<6);		// Toggle LEDs
		_delay_ms (1000);
	}
	return 0;
}


/* Background Reading
 * SPI
 * http://maxembedded.com/2013/11/26/the-spi-of-the-avr/
 * USART
 * http://maxembedded.com/2013/09/30/the-usart-of-the-avr/

 * Bit Operations
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=40348

 * Examples
 * http://svn.lee.org/swarm/trunk/dboard/atmega168/UART.c

 * Manual Sections
 *   : Timers
 * 19: SPI 	161
 * 20: USart0	171

 */

