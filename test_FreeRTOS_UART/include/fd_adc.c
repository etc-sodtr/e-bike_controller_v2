#include <avr/io.h>
#include "../include/fd_adc.h"

void ADC_init(void) {
	ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));	// prescalarea ceasului ADC la 1/128 din frecventa ceasului principal
	ADMUX  = ADC_VREF_TYPE;								// seteaza tensiunea de referinta la valoarea data de ADC_VREF_TYPE
	ADCSRA |= (1<<ADEN);								// activeaza ADC-ul
	ADCSRA |= (1<<ADSC);								// incepe o prima conversie A/D; prima conversie de obicei e mai lenta
}

uint8_t read_adc8(uint8_t adc_input) {
	// citire pe 8 biti
	// rezolutie minima: 1LSB = 19.53mV
	ADMUX = adc_input | ADC_VREF_TYPE | (1<<ADLAR); // selecteaza intrarea ADC
	ADCSRA |= (1<<ADSC);							// incepe conversia A/D
	while (ADCSRA & (1<<ADSC));						// asteapta pana se termina conversia
	return ADCH;									// returneaza valoarea ADC pe 10 biti corespunzatoarea tensiunii de pe intrarea ADC "adc_input"
	/*
				V(adc_input) * 2^8
		ADCH = --------------------
					 V(VREF)
	*/
}
uint16_t read_adc10(uint8_t adc_input) {
	// citire pe 10 biti
	// rezolutie minima: 1LSB = 4.88mV
	ADMUX = adc_input | ADC_VREF_TYPE | (0<<ADLAR); // selecteaza intrarea ADC
	ADCSRA |= (1<<ADSC);							 // incepe conversia A/D
	while (ADCSRA & (1<<ADSC));						 // asteapta pana se termina conversia
	return ADCW;									 // returneaza valoarea ADC pe 10 biti corespunzatoarea tensiunii de pe intrarea ADC "adc_input"
	/*
				V(adc_input) * 2^10
		ADCW = ---------------------
					  V(VREF)
	*/
}