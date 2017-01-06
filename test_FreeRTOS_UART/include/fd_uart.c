#include <avr/io.h>
#include <avr/interrupt.h>
#include "../include/fd_uart.h"

volatile uint8_t rx_buffer_index = 0;
volatile char rx_buffer[UART1_RX_BUFFER_SIZE+1]; // mesaj de maxim 8 caractere lungime, +1 caracter terminator (\n, \0);
volatile uint8_t new_data_received = 0;

/* Functii UART0 */
void uart0_init(unsigned int baudrate) {
	/* seteaza rata BAUD */
	UBRR0H = (unsigned char)(baudrate >> 8);
	UBRR0L = (unsigned char)baudrate;
	
	/* activeaza transmitatorul si receptorul */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	/* Seteaza formatul cadrului: transmisie asincrona, 8b date, fara paritate, 1b stop */
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void uart0_putc(unsigned char data) {
	/* asteapta pana se goleste buffer-ul de transmitere */
	while (!(UCSR0A & (1<<UDRE0))) ;
	
	/* pune caracterul in buffer, transmite caracterul */
	UDR0 = data;
}

void uart0_puts(const char *data) {
	while (*data)				// cat timp exista caractere de transmis
		uart0_putc(*data++);	// transmite caracter
}

unsigned char uart0_getc(void) {
	/* asteapta pana s-au primit date */
	while (!(UCSR0A & (1<<RXC0))) ;
	
	return UDR0;
}

void uart0_flush(void) {
	unsigned char dummy;
	while (UCSR0A & (1<<RXC0)) dummy = UDR0;
}

/* Functii UART1 */
ISR(USART1_RX_vect) {
	if (rx_buffer_index > (UART1_RX_BUFFER_SIZE-1)) {
		uart1_flush();
		rx_buffer[UART1_RX_BUFFER_SIZE] = '\0';
		new_data_received = 0;
	}
	else {
		rx_buffer[rx_buffer_index] = UDR1;
		if (rx_buffer[rx_buffer_index] == '\n') { // \r pentru proteus
			new_data_received = 1;
			rx_buffer[rx_buffer_index] = '\0';
			rx_buffer_index = 0;
		}
		else rx_buffer_index++;
	}
}

void uart1_init(unsigned int baudrate) {
	/* seteaza rata BAUD */
	UBRR1H = (unsigned char)(baudrate >> 8);
	UBRR1L = (unsigned char)baudrate;
	
	/* activeaza modul de viteza dubla */
	UCSR1A = (1<<U2X1);

	/* activeaza transmitatorul si receptorul */
	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);

	/* Seteaza formatul cadrului: transmisie asincrona, 8b date, fara paritate, 1b stop */
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
}
void uart1_putc(unsigned char data) {
	/* asteapta pana se goleste buffer-ul de transmitere */
	while (!(UCSR1A & (1<<UDRE1))) ;
	
	/* pune caracterul in buffer, transmite caracterul */
	UDR1 = data;
}
void uart1_puts(const char *data) {
	while (*data)				// cat timp exista caractere de transmis
		uart1_putc(*data++);	// transmite caracter
}
unsigned char uart1_getc(void) {
	/* asteapta pana s-au primit date */
	while (!(UCSR1A & (1<<RXC1))) ;
	return UDR1;
}

uint8_t uart1_gets(char *data) {
	if (new_data_received == 0) {
		data[0] = '\0';
		return 0;
	}
	else {
		for(uint8_t i = 0; i < UART1_RX_BUFFER_SIZE; i++)
			data[i] = rx_buffer[i];
			new_data_received = 0;
		return 1;
	}
}

void uart1_flush(void) {
	unsigned char dummy;
	while (UCSR1A & (1<<RXC1)) dummy = UDR1;
}