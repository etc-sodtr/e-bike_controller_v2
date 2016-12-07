/*
 * e-bike controller v2 / FreeRTOS tests / blink-a-led
 *
 * Created: 06.12.2016
 * Author : Florin Dumitrache
 */ 

// include AVR headers
#include <avr/io.h>

// include OS
#include "include/FreeRTOS.h"
#include "include/task.h"
#include "include/bitOperations.h"

// tasks
static void blink_Led_PA1(void *pvParameters);
static void blink_Led_PA2(void *pvParameters);
static void blink_Led_PA3(void *pvParameters);
static void blink_Led_PA4(void *pvParameters);

int main(void) {
// initializare porturi
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

	// create blinking task
	xTaskCreate(blink_Led_PA1, "blink_PA1", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // LED pe PORTA1; 1Hz
	xTaskCreate(blink_Led_PA2, "blink_PA2", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // LED pe PORTA2; 2Hz
	xTaskCreate(blink_Led_PA3, "blink_PA3", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // LED pe PORTA3; 3Hz
	xTaskCreate(blink_Led_PA4, "blink_PA4", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // LED pe PORTA4; 4Hz

	// start the scheduler
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

static void blink_Led_PA1(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = 3000;

	(void)pvParameters; // parameters not used

	// forever loop
	while (1) {
		vTaskDelayUntil(&xLastWakeTime, delay/2);
		pin_Toggle(A,1);
	}
}
static void blink_Led_PA2(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = 1500;

	(void)pvParameters; // parameters not used

	// forever loop
	while (1) {
		vTaskDelayUntil(&xLastWakeTime, delay/2);
		pin_Toggle(A,2);
	}
}
static void blink_Led_PA3(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = 1000;

	(void)pvParameters; // parameters not used

	// forever loop
	while (1) {
		vTaskDelayUntil(&xLastWakeTime, delay/2);
		pin_Toggle(A,3);
	}
}
static void blink_Led_PA4(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = 750;

	(void)pvParameters; // parameters not used

	// forever loop
	while (1) {
		vTaskDelayUntil(&xLastWakeTime, delay/2);
		pin_Toggle(A,4);
	}
}
