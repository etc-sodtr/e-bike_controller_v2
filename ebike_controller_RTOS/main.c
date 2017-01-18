/*
 * e-bike controller v2
 * file: main.c
 *
 * Created: 06.01.2017
 * Author : Florin Dumitrache
 */ 

/*** MACRO SECTION ***/
#define F_CPU 16000000UL

#define accel_min_val	240  // valoarea minima primita de la maneta de acceleratie + rezerva de siguranta (evita pornirea accidentala a motorului)
#define accel_max_val	840  // valoarea maxima primita de la maneta de acceleratie
#define OCP_threshold	963  // 30A
#define UVP_threshold	670  // 30.55V
#define OTP_threshold	1007 // 80*C
#define speed_limit		66   // 66 pulsuri/sec = 25km/h

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
#define state_fault		6
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
static void faultChecker(void *pvParameters);
static void speed_limiter(void *pvParameters);
static void UART1_sendData(void *pvParameters);
static void UART1_getData(void *pvParameters);
static void SignalLights(void *pvParameters);
/*** END TASKS SECTION ***/

/*** GLOBAL VARIABLES SECTION ***/

volatile uint16_t acceleration = 0x00;
volatile uint16_t batt_voltage = 0x00;
volatile uint16_t H3_current   = 0x00;
volatile uint16_t MAX6175_temp = 0x00;

volatile uint16_t hall_pulsesPerSec = 0x00;
volatile uint8_t  hall_sensorStates = 0x00;

volatile uint16_t speed_level = 0x00;
volatile uint8_t  speed_whole = 0x00; // pentru afisare viteza - partea intreaga a numarului
volatile uint8_t  speed_fract = 0x00; // pentru afisare viteza - partea fractionara a numarului

volatile uint16_t motor_power         = 0x00;
volatile uint16_t motor_power_limited = 0x00;


volatile uint8_t flags = 0b00000000;
//                         |||||||'-- 0 - stare far:   1 = on; 0 = off
//                         ||||||'--- 1 - stare motor: 1 = on; 0 = off
//                         |||||'---- 2 - stare semnalizare stg: 1 = blink; 0 = off
//                         ||||'----- 3 - stare semnalizare dr:  1 = blink; 0 = off
//                         |||'------ 4 - N/A
//                         ||'------- 5 - N/A
//                         |'-------- 6 - stare sistem: 1 = avarie; 0 = sistem OK
//                         '--------- 7 - debug: 1 = on; 0 = off
/*** END GLOBAL VARIABLES SECTION ***/

/*** MUTEXES SECTION ***/
SemaphoreHandle_t mutex_ADCvalues  = NULL;
SemaphoreHandle_t mutex_speedLevel = NULL;
/*** END MUTEXES SECTION ***/

/*** ISR ROUTINES SECTION ***/
ISR(TIMER0_OVF_vect) {
	// intrerupere generata cu o frecventa de 3kHz
	// utilizata pentru controlul motorului

	TCNT0 = 89;
	hall_sensorStates |= (hall_A_state<<2)|(hall_B_state<<1)|(hall_C_state<<0); // actualizeaza starea curenta a senzorilor

	// verifica nivelul de acceleratie
	if      (acceleration <= accel_min_val) {taskENTER_CRITICAL(); flag_reset(flags,state_motor); taskEXIT_CRITICAL(); motor_power = 0;}
	else if (acceleration >= accel_max_val) {taskENTER_CRITICAL(); flag_set(flags,state_motor);	  taskEXIT_CRITICAL(); motor_power = 1200;}
	else                                    {taskENTER_CRITICAL(); flag_set(flags,state_motor);   taskEXIT_CRITICAL(); motor_power = 2*(acceleration-accel_min_val);}
	
	BLDC_HPWM_LON(hall_sensorStates, flag_isSet(flags,state_motor), motor_power_limited);

	if ((hall_sensorStates & 0x44) == 0x04) hall_pulsesPerSec += 1;
	hall_sensorStates = (hall_sensorStates << 4);

	if (flag_isSet(flags,state_debug)) {
		PORTA &= 0x01;
		PORTA |= (hall_sensorStates << 1); // afiseaza starea curenta a senzorilor Hall pe PORTA
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
	timer0_init();  // overflow cu frecventa de aprox. 3kHZ
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

	xTaskCreate(speed_limiter,  "task5", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(ADC_readInputs, "task1", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(faultChecker,   "task6", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(UART1_sendData, "task2", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(UART1_getData,  "task3", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(SignalLights,   "task4", configMINIMAL_STACK_SIZE, NULL, 6, NULL);

	// start the scheduler
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

static void ADC_readInputs(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/10;
	
	(void)pvParameters;
	mutex_ADCvalues = xSemaphoreCreateMutex();
	
	// variabile locale
	uint16_t MAX6175_temp_local = 0x00;
	uint16_t H3_current_local   = 0x00;
	uint16_t batt_voltage_local = 0x00;
	uint16_t acceleration_local = 0x00;
	
	while (1) {
		MAX6175_temp_local = read_adc10(0);
		H3_current_local   = read_adc10(1);
		batt_voltage_local = read_adc10(2);
		acceleration_local = read_adc10(7);
		
		if (mutex_ADCvalues != NULL) {
			if (xSemaphoreTake(mutex_ADCvalues,(TickType_t)10) == pdTRUE) {
				MAX6175_temp = MAX6175_temp_local;
				H3_current   = H3_current_local;
				batt_voltage = batt_voltage_local;
				acceleration = acceleration_local;
				xSemaphoreGive(mutex_ADCvalues);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void speed_limiter(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ/6; // frecventa de 6 Hz
	
	(void)pvParameters;
	mutex_speedLevel = xSemaphoreCreateMutex();
	
	while (1) {
		if (mutex_speedLevel != NULL) {
			if (xSemaphoreTake(mutex_speedLevel,(TickType_t)10) == pdTRUE) {
				speed_level += hall_pulsesPerSec;
				xSemaphoreGive(mutex_speedLevel);
			}
		}
		taskENTER_CRITICAL();
		if (hall_pulsesPerSec > (speed_limit/6+1)) // 11 pulsuri + 1 puls(corectie)s
			motor_power_limited = motor_power>>1; // foarte rudimentar, dar functioneaza
		else
			motor_power_limited = motor_power;
		taskEXIT_CRITICAL();
		hall_pulsesPerSec = 0x00; // reseteaza numarul de pulsuri pentru urmatoarea masuratoare
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void faultChecker(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ;
	
	(void)pvParameters;
	
	uint8_t UVP_counter = 0x00; // contor protectie subtensiune
	uint8_t OCP_counter = 0x00; // contor protectie supracurent
	uint8_t OTP_counter = 0x00; // contor protectie supratemperatura
	
	while (1) {
		if (~(flag_isSet(flags,state_fault))) {
			// protectie subtensiune
			if (mutex_ADCvalues != NULL) {
				if (xSemaphoreTake(mutex_ADCvalues,(TickType_t)10) == pdTRUE) {
					if (batt_voltage > UVP_threshold) UVP_counter = 0x00;
					else UVP_counter += 1; // daca tensiunea bateriei este sub 30,5V incrementeaza contorul
					xSemaphoreGive(mutex_ADCvalues);
				}
			}

			// protectie supracurent
			if (mutex_ADCvalues != NULL) {
				if (xSemaphoreTake(mutex_ADCvalues,(TickType_t)10) == pdTRUE) {
					if (H3_current < OCP_threshold) OCP_counter = 0x00;
					else OCP_counter += 1; // daca curentul consumat este peste 30A incrementeaza contorul
					xSemaphoreGive(mutex_ADCvalues);
				}
			}

			// protectie supratemperatura
			if (mutex_ADCvalues != NULL) {
				if (xSemaphoreTake(mutex_ADCvalues,(TickType_t)10) == pdTRUE) {
					if (MAX6175_temp < OTP_threshold) OTP_counter = 0x00;
					else OTP_counter += 1; // daca curentul consumat este peste 30A incrementeaza contorul
					xSemaphoreGive(mutex_ADCvalues);
				}
			}

			if (UVP_counter > 4) {             // daca tensiunea bateriei este sub 30,5V timp de 5 secunde
				taskENTER_CRITICAL();
				flag_set(flags,state_fault);   // activeaza flag-ul de avarie
				flag_reset(flags,state_motor); // opreste motorul
				uart1_puts("log UVP condition met!\n");
			}
			if (OCP_counter > 2) {             // daca curentul consumat este peste 30A timp de 3 secunde
				taskENTER_CRITICAL();
				flag_set(flags,state_fault);   // activeaza flag-ul de avarie
				flag_reset(flags,state_motor); // opreste motorul
				taskEXIT_CRITICAL();
				uart1_puts("log OCP condition met!\n");
			}
			if (OTP_counter > 59) {            // daca temperatura este peste 85*C timp de 60 secunde
				taskENTER_CRITICAL();
				flag_set(flags,state_fault);   // activeaza flag-ul de avarie
				flag_reset(flags,state_motor); // opreste motorul
				taskEXIT_CRITICAL();
				uart1_puts("log OTP condition met!\n");
			}
		}
		vTaskDelayUntil(&xLastWakeTime, delay);
	}
}

static void UART1_sendData(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t delay = configTICK_RATE_HZ; // delay 1 secunda
	char tx_buffer[UART1_TX_BUFFER_SIZE];
	
	(void)pvParameters;
	// variabile locale
	float    speed = 0.0;
	uint16_t MAX6175_temp_local = 0x00;
	uint16_t H3_current_local   = 0x00;
	uint16_t batt_voltage_local = 0x00;
	uint16_t acceleration_local = 0x00;
	
	while (1) {
		pin_SetHI(D,0);
		
		uart1_puts("L 1\n"); // mesaj tip "heartbeat"
		if (flag_isSet(flags,state_fault)) uart1_puts("A 1\n");
		
		if (mutex_speedLevel != NULL) {
			if (xSemaphoreTake(mutex_speedLevel,(TickType_t)10) == pdTRUE) {
				speed = (float)speed_level;
				speed_level = 0;
				xSemaphoreGive(mutex_speedLevel);
			}
		}
		
		speed = (float)(speed*0.3734); // speed = (hall_pulsesPerSec/20)*(pi*26)*(0,0254*3,6)
		speed_whole = (uint8_t)speed;
		speed_fract = (uint8_t)((speed - (float)speed_whole)*100);
		sprintf(tx_buffer,"SW %d\n",speed_whole); uart1_puts(tx_buffer);
		sprintf(tx_buffer,"SF %d\n",speed_fract); uart1_puts(tx_buffer);
		
		if (mutex_ADCvalues != NULL) {
			if (xSemaphoreTake(mutex_ADCvalues,(TickType_t)10) == pdTRUE) {
				MAX6175_temp_local = MAX6175_temp;
				H3_current_local   = H3_current;
				batt_voltage_local = batt_voltage;
				acceleration_local = acceleration;
				xSemaphoreGive(mutex_ADCvalues);
			}
		}

		sprintf(tx_buffer,"P %d\n",acceleration_local); uart1_puts(tx_buffer);
		sprintf(tx_buffer,"B %d\n",batt_voltage_local); uart1_puts(tx_buffer);
		sprintf(tx_buffer,"T %d\n",MAX6175_temp_local); uart1_puts(tx_buffer);
		sprintf(tx_buffer,"C %d\n",H3_current_local);   uart1_puts(tx_buffer);

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
				taskENTER_CRITICAL();
				flag_toggle(flags,state_headlight);
				taskEXIT_CRITICAL();
				pin_SetState(A,0,flag_isSet(flags,state_headlight));
				sprintf(tx_buffer,"F %d\n",flag_isSet(flags,state_headlight));
				uart1_puts(tx_buffer);
			}
			if (strcmp(rx_buffer,"SL") == 0) {
				taskENTER_CRITICAL();
				flag_reset(flags,state_sigR);
				flag_toggle(flags,state_sigL);
				taskEXIT_CRITICAL();
			}
			if (strcmp(rx_buffer,"SR") == 0) {
				taskENTER_CRITICAL();
				flag_reset(flags,state_sigL);
				flag_toggle(flags,state_sigR);
				taskEXIT_CRITICAL();
			}
			if (strcmp(rx_buffer,"DBG_1") == 0) {
				taskENTER_CRITICAL();
				flag_set(flags,state_debug);
				taskEXIT_CRITICAL();
			}
			if (strcmp(rx_buffer,"DBG_0") == 0) {
				taskENTER_CRITICAL();
				flag_reset(flags,state_debug);
				taskEXIT_CRITICAL();
			}
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
