/*
 * e-bike controller v2
 * file: main.c
 *
 * Created: 06.01.2017
 * Author : Florin Dumitrache
 */ 

/*** MACRO SECTION ***/
#define F_CPU 16000000UL

#define accel_min_val	240 // valoarea minima primita de la maneta de acceleratie + rezerva de siguranta (evita pornirea accidentala a motorului)
#define accel_max_val	840 // valoarea maxima primita de la maneta de acceleratie
#define OCP_threshold	963 // 30A
#define UVP_threshold	670 // 30.55V
#define speed_limit		66  // 25km/h

// detalii despre hall_A_state, hall_B_state si hall_C_state in fd_bldc.c
#define hall_A_state (pin_ReadStateLO(E,6) & (pin_ReadStateHI(D,1) | pin_ReadStateLO(E,7)))
#define hall_B_state (pin_ReadStateLO(D,1) & (pin_ReadStateHI(E,6) | pin_ReadStateLO(E,7)))
#define hall_C_state (pin_ReadStateHI(E,7))

#define flag_isSet(reg,bit)  (reg &   (1<<bit))
#define flag_toggle(reg,bit) (reg ^=  (1<<bit))
#define flag_set(reg,bit)    (reg |=  (1<<bit))
#define flag_reset(reg,bit)  (reg &= ~(1<<bit))

#define state_headlight	0
#define state_motor		1
#define state_sigL		2
#define state_sigR		3
#define state_debug		7
/*** END MACRO SECTION  ***/

/*** AVR/GCC HEADERS SECTION ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*** END AVR/GCC HEADERS SECTION ***/

/*** FreeRTOS HEADERS SECTION ***/
#include "include/FreeRTOS.h"
#include "include/task.h"
#include "include/semphr.h"
/*** END FreeRTOS HEADERS SECTION ***/

/*** APPLICATION HEADERS SECTION ***/
#include "include/bitOperations.h"
#include "include/fd_adc.h"
#include "include/fd_bldc.h"
#include "include/fd_timer.h"
#include "include/fd_uart.h"
/*** END APPLICATION HEADERS SECTION ***/

/*** TASKS SECTION ***/
static void ADC_readInputs(void *pvParameters);
static void UART1_sendData(void *pvParameters);
static void UART1_getData(void *pvParameters);
static void SignalLights(void *pvParameters);
/*** END TASKS SECTION ***/

/*** GLOBAL VARIABLES SECTION ***/

uint16_t acceleration		= 0x00;
uint16_t batt_voltage		= 0x00;
uint16_t H3_current			= 0x00;
uint16_t hall_pulsesPerSec	= 0x00;
uint8_t	 hall_sensorStates	= 0x00;
uint16_t MAX6175_temp		= 0x00;

uint8_t UVP_counter = 0x00; // contor protectie subtensiune
uint8_t OCP_counter = 0x00; // contor protectie supracurent

uint16_t speed_level = 0x00;
uint16_t power_level = 0x00;

float   speed       = 0.0;
uint8_t speed_whole = 0x00; // pentru afisare viteza - partea intreaga a numarului
uint8_t speed_fract = 0x00; // pentru afisare viteza - partea fractionara a numarului

uint8_t	 flags = 0b00000000;
//				   |||||||'-- 0 - stare far:   1 = on; 0 = off;
//				   ||||||'--- 1 - stare motor: 1 = on; 0 = off;
//				   |||||'---- 2 - stare semnalizare stg: 1 = blink; 0 = off;
//				   ||||'----- 3 - stare semnalizare dr:  1 = blink; 0 = off;
//				   |||'------ 4 - N/A
//				   ||'------- 5 - N/A
//				   |'-------- 6 - N/A
//				   '--------- 7 - debug: 1 = on; 0 = off
/*** END GLOBAL VARIABLES SECTION ***/

/*** MUTEXES SECTION ***/
SemaphoreHandle_t mutex_ADC_values = NULL;
/*** END MUTEXES SECTION ***/

/*** ISR ROUTINES SECTION ***/
ISR(TIMER0_OVF_vect) {
	// intrerupere generata cu o frecventa de 3kHz
	// utilizata pentru controlul motorului

	TCNT0 = 89;
	hall_sensorStates |= (hall_A_state<<2)|(hall_B_state<<1)|(hall_C_state<<0); // actualizeaza starea curenta a senzorilor

	// verifica nivelul de acceleratie
	if      (acceleration <= accel_min_val) {flag_reset(flags,state_motor); power_level = 0;}
	else if (acceleration >= accel_max_val) {flag_set(flags,state_motor);	power_level = 1200;}
	else                                    {flag_set(flags,state_motor);   power_level = 2*(acceleration-accel_min_val);}
	
	BLDC_HPWM_LON(hall_sensorStates, flag_isSet(flags,state_motor), power_level);

	if ((hall_sensorStates & 0x44) == 0x04) hall_pulsesPerSec += 1;
	hall_sensorStates = (hall_sensorStates << 4);

	speed = (float)(hall_pulsesPerSec*0.3734); // speed = (hall_pulsesPerSec/20)*(pi*26)*(0,0254*3,6)
	speed_whole = (uint8_t)speed;
	speed_fract = (uint8_t)((speed - (float)speed_whole)*100);

	if (flag_isSet(flags,state_debug)) {
		PORTA &= 0x01;
		PORTA |= (hall_sensorStates << 1); // afiseaza starea curenta a senzorilor pe leduri
	}
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
	timer0_init(); // overflow cu frecventa de aprox. 3kHZ
	timer3_init();  // PWM la 16kHz
	uart1_init(16); // UART_BAUD_SELECT(19200,16000000) = 16
	sei();
	_delay_ms(1000);
	uart1_puts("log INIT OK!\n");
	uart1_puts("B 0\n");
	uart1_puts("T 0\n");
	uart1_puts("F 0\n");
	uart1_puts("SW 0\n");
	uart1_puts("SF 0\n");
	pin_SetLO(D,0);
/*** END INTERNAL MODULES INITIALIZATION SECTION ***/

	xTaskCreate(ADC_readInputs, "task1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(UART1_sendData, "task2", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(UART1_getData,  "task3", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(SignalLights,   "task4", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

	// start the scheduler
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

static void ADC_readInputs(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/10;
	
	(void)pvParameters;
	mutex_ADC_values = xSemaphoreCreateMutex();
	
	while (1) {
		if (mutex_ADC_values != NULL) {
			if (xSemaphoreTake(mutex_ADC_values,(TickType_t)10) == pdTRUE) {
				MAX6175_temp = read_adc10(0);
				H3_current   = read_adc10(1);
				batt_voltage = read_adc10(2);
				acceleration = read_adc10(7);
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
	char tx_buffer[UART1_TX_BUFFER_SIZE];
	
	(void)pvParameters;
	
	while (1) {
		pin_SetHI(D,0);
		
		uart1_puts("L 1\n"); // mesaj tip "heartbeat"
		sprintf(tx_buffer,"SW %d\n",speed_whole); uart1_puts(tx_buffer);
		sprintf(tx_buffer,"SF %d\n",speed_fract); uart1_puts(tx_buffer);
		speed_level = hall_pulsesPerSec;
		hall_pulsesPerSec = 0x00; // reseteaza numarul de pulsuri pentru urmatoarea masuratoare
		
		if (mutex_ADC_values != NULL) {
			if (xSemaphoreTake(mutex_ADC_values,(TickType_t)10) == pdTRUE) {
				sprintf(tx_buffer,"P %d\n",acceleration); uart1_puts(tx_buffer);
				sprintf(tx_buffer,"B %d\n",batt_voltage); uart1_puts(tx_buffer);
				sprintf(tx_buffer,"T %d\n",MAX6175_temp); uart1_puts(tx_buffer);
				sprintf(tx_buffer,"C %d\n",H3_current);   uart1_puts(tx_buffer);
				xSemaphoreGive(mutex_ADC_values);
			}
		}
		pin_SetLO(D,0);
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void UART1_getData(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/10;
	char rx_buffer[UART1_RX_BUFFER_SIZE];
	char tx_buffer[UART1_TX_BUFFER_SIZE];
	
	(void)pvParameters;
	
	while (1) {
		if (uart1_gets(rx_buffer) == 0) ;
		else {
			if (strcmp(rx_buffer,"FAR") == 0) {
				flag_toggle(flags,state_headlight);
				pin_SetState(A,0,flag_isSet(flags,state_headlight));
				sprintf(tx_buffer,"F %d\n",flag_isSet(flags,state_headlight));
				uart1_puts(tx_buffer);
			}
			if (strcmp(rx_buffer,"SL") == 0) {
				flag_reset(flags,state_sigR);
				flag_toggle(flags,state_sigL);
			}
			if (strcmp(rx_buffer,"SR") == 0) {
				flag_reset(flags,state_sigL);
				flag_toggle(flags,state_sigR);
			}
			if (strcmp(rx_buffer,"DBG_1") == 0) flag_set(flags,state_debug);
			if (strcmp(rx_buffer,"DBG_0") == 0) flag_reset(flags,state_debug);
		}
		rx_buffer[0] = '\0';
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void SignalLights(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/2;
	
	(void)pvParameters;
	
	while (1) {
		if (flag_isSet(flags,state_debug)) ;
		else {
			if (flag_isSet(flags,state_sigL)) {
				PORTA &= 0b11111001;
				PORTA ^= 0b11000000;
			}
			else if (flag_isSet(flags,state_sigR)) {
				PORTA &= 0b00111111;
				PORTA ^= 0b00000110;
			}
			else PORTA &= 0b00000001;
		}
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}
