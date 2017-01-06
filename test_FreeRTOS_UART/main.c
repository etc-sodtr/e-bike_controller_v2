/*
 * e-bike controller v2 / FreeRTOS tests / UART communication
 *
 * Created: 13.12.2016
 * Author : Florin Dumitrache
 */ 

/*** MACRO SECTION ***/
#define F_CPU 16000000UL

#define flag_isSet(reg,bit)  (reg &   (1<<bit))
#define flag_toggle(reg,bit) (reg ^=  (1<<bit))
#define flag_set(reg,bit)    (reg |=  (1<<bit))
#define flag_reset(reg,bit)  (reg &= ~(1<<bit))
/*** END MACRO SECTION  ***/

/*** AVR HEADERS SECTION ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/*** END AVR HEADERS SECTION ***/

/*** FreeRTOS HEADERS SECTION ***/
#include "include/FreeRTOS.h"
#include "include/task.h"
#include "include/semphr.h"
/*** END FreeRTOS HEADERS SECTION ***/

/*** APPLICATION HEADERS SECTION ***/
#include "include/bitOperations.h"
#include "include/fd_adc.h"
#include "include/fd_timer.h"
#include "include/fd_uart.h"
/*** END APPLICATION HEADERS SECTION ***/

/*** TASKS SECTION ***/
static void PortA_bargraph(void *pvParameters);
static void ADC_readInputs(void *pvParameters);
static void UART1_sendData(void *pvParameters);
static void UART1_receiveData(void *pvParameters);
/*** END TASKS SECTION ***/

/*** GLOBAL VARIABLES SECTION ***/
char tx_buffer[UART1_TX_BUFFER_SIZE];
char rx_buffer[UART1_RX_BUFFER_SIZE];

uint8_t PORTA_lvl  = 0x60;
volatile uint8_t PORTA_LEDS = 0x00;

uint16_t batt_voltage = 0x00;
uint16_t MAX6175_temp = 0x00;
/*** END GLOBAL VARIABLES SECTION ***/

/*** MUTEXES SECTION ***/
SemaphoreHandle_t mutex_ADC_values = NULL;
SemaphoreHandle_t mutex_PORTA_lvl  = NULL;
/*** END MUTEXES SECTION ***/

/*** ISR ROUTINES SECTION ***/
ISR(TIMER2_COMP_vect) {
	PORTA = 0x00;
}
ISR(TIMER2_OVF_vect) {
	PORTA = PORTA_LEDS;
}
/*** END ISR ROUTINES SECTION ***/

int main(void) {

/*** PORTS INITIALIZATION SECTION ***/
// Port A /   EXT_PA7  /  EXT_PA6  /  EXT_PA5  /  EXT_PA4  /  EXT_PA3  /  EXT_PA2  /  EXT_PA1  /  Far_out
	DDRA  = (1<<DDRA7) |(1<<DDRA6) |(1<<DDRA5) |(1<<DDRA4) |(1<<DDRA3) |(1<<DDRA2) |(1<<DDRA1) |(1<<DDRA0);
	PORTA = (0<<PORTA7)|(0<<PORTA6)|(0<<PORTA5)|(0<<PORTA4)|(0<<PORTA3)|(0<<PORTA2)|(0<<PORTA1)|(0<<PORTA0);

// Port B /    OC1C    /   OC1B    /   OC1A    /    OC0    /           /           /     -     /
	DDRB  = (1<<DDRB7) |(1<<DDRB6) |(1<<DDRB5) |(0<<DDRB4) |(0<<DDRB3) |(0<<DDRB2) |(0<<DDRB1) |(0<<DDRB0);
	PORTB = (0<<PORTB7)|(0<<PORTB6)|(0<<PORTB5)|(0<<PORTB4)|(0<<PORTB3)|(0<<PORTB2)|(0<<PORTB1)|(0<<PORTB0);

// Port C /   EXT_PC7  /  EXT_PC6  /  EXT_PC5  /  EXT_PC4  /  EXT_PC3  /  EXT_PC2  /  EXT_PC1  /  EXT_PC0
	DDRC  = (0<<DDRC7) |(0<<DDRC6) |(0<<DDRC5) |(0<<DDRC4) |(0<<DDRC3) |(0<<DDRC2) |(0<<DDRC1) |(0<<DDRC0);
	PORTC = (0<<PORTC7)|(0<<PORTC6)|(0<<PORTC5)|(0<<PORTC4)|(0<<PORTC3)|(1<<PORTC2)|(1<<PORTC1)|(1<<PORTC0);

// Port D /    Sig_R   /   Sig_L   /   Brake   /  Far_in   /   TxD1    /   RxD1    /  Hall_A   / Led_test
	DDRD  = (0<<DDRD7) |(0<<DDRD6) |(0<<DDRD5) |(0<<DDRD4) |(1<<DDRD3) |(0<<DDRD2) |(0<<DDRD1) |(1<<DDRD0);
	PORTD = (1<<PORTD7)|(1<<PORTD6)|(1<<PORTD5)|(1<<PORTD4)|(0<<PORTD3)|(0<<PORTD2)|(1<<PORTD1)|(0<<PORTD0);

// Port E /   Hall_C   /  Hall_B   /   OC3C    /   OC3B    /   OC3A    /           /           /
	DDRE  = (0<<DDRE7) |(0<<DDRE6) |(1<<DDRE5) |(1<<DDRE4) |(1<<DDRE3) |(0<<DDRE2) |(0<<DDRE1) |(0<<DDRE0);
	PORTE = (1<<PORTE7)|(1<<PORTE6)|(0<<PORTE5)|(0<<PORTE4)|(0<<PORTE3)|(0<<PORTE2)|(0<<PORTE1)|(0<<PORTE0);

// Port F /  Throttle  /   ADC6    /   ADC5    /   ADC4    /   ADC3    /   VBAT    /  ASENSE   /   TEMP
	DDRF  = (0<<DDRF7) |(0<<DDRF6) |(0<<DDRF5) |(0<<DDRF4) |(0<<DDRF3) |(0<<DDRF2) |(0<<DDRF1) |(0<<DDRF0);
	PORTF = (0<<PORTF7)|(0<<PORTF6)|(0<<PORTF5)|(0<<PORTF4)|(0<<PORTF3)|(0<<PORTF2)|(0<<PORTF1)|(0<<PORTF0);

// Port G neconectat
	DDRG  = 0x00;
	PORTG = 0x00;
/*** END PORTS INITIALIZATION SECTION ***/

/*** INTERNAL MODULES INITIALIZATION SECTION ***/
	pin_SetHI(D,0);
	ADC_init();
	timer2_init();
	uart1_init(16); // UART_BAUD_SELECT(19200,16000000) = 16
	sei();
	_delay_ms(1000);
	uart1_puts("log INIT OK!\n");
	
	uart1_puts("T 0\n"); // reseteaza temperatura
	uart1_puts("B 0\n"); // reseteaza tensiunea bateriei
	uart1_puts("LVL 96\n");
	uart1_puts("PWM 15\n");
	
	pin_SetLO(D,0);
/*** END INTERNAL MODULES INITIALIZATION SECTION ***/

	xTaskCreate(PortA_bargraph,    "task1", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // LEDuri pe PORTA
	xTaskCreate(ADC_readInputs,    "task2", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(UART1_sendData,    "task3", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(UART1_receiveData, "task4", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

	// start the scheduler
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

static void PortA_bargraph(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/10;
	
	(void)pvParameters;
	mutex_PORTA_lvl = xSemaphoreCreateMutex();
	
	while (1) {
		if (mutex_PORTA_lvl != NULL) {
			if (xSemaphoreTake(mutex_PORTA_lvl,(TickType_t)10) == pdTRUE) {
				if      (PORTA_lvl >= 224)	PORTA_LEDS = 0b11111110;
				else if (PORTA_lvl >= 192)	PORTA_LEDS = 0b11111100;
				else if (PORTA_lvl >= 160)	PORTA_LEDS = 0b11111000;
				else if (PORTA_lvl >= 128)	PORTA_LEDS = 0b11110000;
				else if (PORTA_lvl >= 96)	PORTA_LEDS = 0b11100000;
				else if (PORTA_lvl >= 64)	PORTA_LEDS = 0b11000000;
				else if (PORTA_lvl >= 32)	PORTA_LEDS = 0b10000000;
				else						PORTA_LEDS = 0b00000000;
				xSemaphoreGive(mutex_PORTA_lvl);
			}
			else ;
		}
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void ADC_readInputs(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/2;
	
	(void)pvParameters;
	mutex_ADC_values = xSemaphoreCreateMutex();
	
	while (1) {
		if (mutex_ADC_values != NULL) {
			if (xSemaphoreTake(mutex_ADC_values,(TickType_t)10) == pdTRUE) {
				MAX6175_temp = read_adc10(0);
				batt_voltage = read_adc10(2);
				xSemaphoreGive(mutex_ADC_values);
			}
			else ;
		}
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void UART1_sendData(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ; // delay 1 secunda
	char tx_buffer[16];
	
	(void)pvParameters;
	
	while (1) {
		pin_SetHI(D,0);
		if (mutex_ADC_values != NULL) {
			if (xSemaphoreTake(mutex_ADC_values,(TickType_t)10) == 1) {
				sprintf(tx_buffer,"B %d\n",batt_voltage);
				uart1_puts(tx_buffer);
				sprintf(tx_buffer,"T %d\n",MAX6175_temp);
				uart1_puts(tx_buffer);
				xSemaphoreGive(mutex_ADC_values);
			}
			else ;
		}
		pin_SetLO(D,0);
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void UART1_receiveData(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/10;
	char temp[10];
	
	(void)pvParameters;
	
	while (1) {
		if (uart1_gets(rx_buffer) == 0) ;
		else {
			if ((rx_buffer[0]=='L')&(rx_buffer[1]=='V')&(rx_buffer[2]=='L')) {
				// seteaza cate LED-uri sunt aprinse pe portul A
				strncpy(temp, &rx_buffer[4], 3);
				if (mutex_PORTA_lvl != NULL) {
					if (xSemaphoreTake(mutex_PORTA_lvl,(TickType_t)10) == pdTRUE) {
						PORTA_lvl = atoi(temp);
						xSemaphoreGive(mutex_PORTA_lvl);
					}
					else ;
				}
			}
			if ((rx_buffer[0]=='P')&(rx_buffer[1]=='W')&(rx_buffer[2]=='M')) {
				// seteaza luminozitatea LED-urilor de pe portul A
				strncpy(temp, &rx_buffer[4], 3);
				OCR2 = atoi(temp);
			}
		}
		rx_buffer[0] = '\0';
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}