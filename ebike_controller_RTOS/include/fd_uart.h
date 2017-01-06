/*
 * e-bike controller v1 / v2
 * file: fd_uart.h
 *
 * Created: 31.03.2016
 * Author : Florin Dumitrache
 */

#ifndef UART_H
#define UART_H

#define UART_BAUD_SELECT(baudRate,f_osc) ((f_osc / (16UL * baudRate)) - 1)
#define UART1_RX_BUFFER_SIZE 32
#define UART1_TX_BUFFER_SIZE 16

/* Functii UART0 */
extern void uart0_init(unsigned int baudrate);
extern void uart0_putc(unsigned char data);
extern void uart0_puts(const char *data);
extern unsigned char uart0_getc(void);
extern void uart0_flush(void);

/* Functii UART1 */
extern void uart1_init(unsigned int baudrate);
extern void uart1_putc(unsigned char data);
extern void uart1_puts(const char *data);
extern unsigned char uart1_getc(void);
extern uint8_t uart1_gets(char *data);
extern void uart1_flush(void);

#endif // UART_H 

