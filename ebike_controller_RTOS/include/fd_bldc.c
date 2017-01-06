/*
 * e-bike controller v1 / v2
 * file: fd_bldc.c
 *
 * Created: 10.04.2016
 * Author : Florin Dumitrache
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "../include/bitOperations.h"
#include "../include/fd_bldc.h"

/*
	.-----------------------------------------------------------------.
	|        semnale senzori motor        ||     semnale necesare     |
	| U(galben) | V(albastru) | W (verde) || hall_A | hall_B | hall_C |
	|     0     |       1     |     0     ||   0    |   1    |   0    |
	|     0     |       1     |     1     ||   0    |   1    |   1    |		=> hall_A = U & ~V | ~V & ~W = ~V & (U | ~W)
	|     1     |       1     |     1     ||   0    |   0    |   1    |		=> hall_B = ~U & V | ~U & ~W = ~U & (V | ~W)
	|     1     |       0     |     1     ||   1    |   0    |   1    |		=> hall_C = W
	|     1     |       0     |     0     ||   1    |   0    |   0    |
	|     0     |       0     |     0     ||   1    |   1    |   0    |
	'-----------------------------------------------------------------'
*/

void BLDC_HPWM_LON(uint8_t hall, uint8_t enable, uint16_t duty) {
	
	// hall = 0000 0ABC;
	uint8_t hall_en = (enable) ? (hall & 0x0F) : 0x00;
	
	OCR3A = duty;
	OCR3B = duty;
	OCR3C = duty;
	
	/*
		*** tabela semnale Hall si secventa de tranzistoare ce trebuie actionate pentru comutatia dreptunghiulara H_PWM-L_ON
		*** semnalele LOW trebuie trimise inversat !!!

		.--------------------------------------------------------------------------------------------------------------------.
		| sector | Hall A | Hall B | Hall C | hall_en || M1-M2 | M3-M4 | M5-M6 || A_high-A_low | B_high-B_low | C_high-C_low |
		|   1    |   0    |   0    |   1    |    1    ||  0-0  |  P-0  |  0-1  ||      0-1     |      P-1     |      0-0     |
		|   2    |   0    |   1    |   1    |    3    ||  P-0  |  0-0  |  0-1  ||      P-1     |      0-1     |      0-0     |
		|   3    |   0    |   1    |   0    |    2    ||  P-0  |  0-1  |  0-0  ||      P-1     |      0-0     |      0-1     |
		|   4    |   1    |   1    |   0    |    6    ||  0-0  |  0-1  |  P-0  ||      0-1     |      0-0     |      P-1     |
		|   5    |   1    |   0    |   0    |    4    ||  0-1  |  0-0  |  P-0  ||      0-0     |      0-1     |      P-1     |
		|   6    |   1    |   0    |   1    |    5    ||  0-1  |  P-0  |  0-0  ||      0-0     |      P-1     |      0-1     |
		'--------------------------------------------------------------------------------------------------------------------'
	*/
	
	switch(hall_en) {
		// dezactiveaza iesirile OCR3x
		// reseteaza iesirile AH, BH, CH din portul E
		// reseteaza iesirile AL, BL, CL din portul B
		// asteapta <deadtime_us> microsecunde
		// seteaza iesirile AL, BL, CL din portul B corespunzator sectorului
		// activeaza iesirea OCR3x aferenta sectorului
		case 1:  {Timer3_OCR3x_disable; PORTE_reset_PWM_outputs; PORTB_reset_PWM_outputs; _delay_us(deadtime_us); PORTB &= ~(1<<PWMC_LOW); Timer3_OCR3B_enable_nonInv; break;}
		case 2:  {Timer3_OCR3x_disable; PORTE_reset_PWM_outputs; PORTB_reset_PWM_outputs; _delay_us(deadtime_us); PORTB &= ~(1<<PWMB_LOW); Timer3_OCR3A_enable_nonInv; break;}
		case 3:  {Timer3_OCR3x_disable; PORTE_reset_PWM_outputs; PORTB_reset_PWM_outputs; _delay_us(deadtime_us); PORTB &= ~(1<<PWMC_LOW); Timer3_OCR3A_enable_nonInv; break;}
		case 4:  {Timer3_OCR3x_disable; PORTE_reset_PWM_outputs; PORTB_reset_PWM_outputs; _delay_us(deadtime_us); PORTB &= ~(1<<PWMA_LOW); Timer3_OCR3C_enable_nonInv; break;}
		case 5:  {Timer3_OCR3x_disable; PORTE_reset_PWM_outputs; PORTB_reset_PWM_outputs; _delay_us(deadtime_us); PORTB &= ~(1<<PWMA_LOW); Timer3_OCR3B_enable_nonInv; break;}
		case 6:  {Timer3_OCR3x_disable; PORTE_reset_PWM_outputs; PORTB_reset_PWM_outputs; _delay_us(deadtime_us); PORTB &= ~(1<<PWMB_LOW); Timer3_OCR3C_enable_nonInv; break;}
		default: {Timer3_OCR3x_disable; PORTE_reset_PWM_outputs; PORTB_reset_PWM_outputs; break;}
	}
}

void bldc_PWM_ON(uint8_t hall, uint8_t enable, uint16_t duty) {
	
	// hall = 0000 0ABC;
	uint8_t hall_en = (enable) ? (hall & 0x0F) : 0x00;
	
	OCR3A = duty;
	OCR3B = duty;
	OCR3C = duty;
	
	/*
		*** tabela semnale Hall si secventa de tranzistoare ce trebuie actionate pentru comutatia dreptunghiulara PWM-ON
		*** semnalele LOW trebuie trimise inversat !!!

		.--------------------------------------------------------------------------------------------------------------------.
		| sector | Hall A | Hall B | Hall C | hall_en || M1-M2 | M3-M4 | M5-M6 || A_high-A_low | B_high-B_low | C_high-C_low |
		|   1    |   0    |   0    |   1    |    1    ||  1-0  | NP-P  |  0-0  ||      1-1     |     NP-NP    |      0-1     |
		|   2    |   0    |   1    |   1    |    3    ||  0-0  |  0-1  |  P-NP ||      0-1     |      0-0     |      P-P     |
		|   3    |   0    |   1    |   0    |    2    || NP-P  |  0-0  |  1-0  ||     NP-NP    |      0-1     |      1-1     |
		|   4    |   1    |   1    |   0    |    6    ||  0-1  |  P-NP |  0-0  ||      0-1     |      P-P     |      0-1     |
		|   5    |   1    |   0    |   0    |    4    ||  0-0  |  1-0  | NP-P  ||      0-0     |      1-1     |     NP-NP    |
		|   6    |   1    |   0    |   1    |    5    ||  P-NP |  0-0  |  0-1  ||      P-P     |      0-1     |      0-0     |
		'--------------------------------------------------------------------------------------------------------------------'
	*/
	
	switch(hall_en) {
		
		case 1:  {break;}
		case 2:  {break;}
		case 3:  {break;}
		case 4:  {break;}
		case 5:  {break;}
		case 6:  {break;}
		default: {break;}
	}
}

void bldc_shutdown(void) {
	TCCR3A = (1<<WGM31);
	PORTE  = 0b11000100;
	PORTB  = 0b11100000;
}