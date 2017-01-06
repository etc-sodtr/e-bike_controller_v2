/*
 * e-bike controller v1 / v2
 * file: fd_timer.c
 *
 * Created: 31.03.2016
 * Author : Florin Dumitrache
 */

#include <avr/io.h>
#include "../include/fd_timer.h"

void timer0_init(void) {
	// timer folosit pentru controlul motorului
	// mod 0 (normal)
	// frecventa ceas Timer0 = 500kHz (prescaler = 8)
	// overflow interrupt la fiecare (256-TCNT0)/f_TMR0 = 250/500000 = 334us
	// =>frecventa overflow Timer0 ~= 3kHz
	
	TCNT0  = 89;
	TCCR0  = (0<<FOC0)|(0<<WGM01)|(0<<COM01)|(0<<COM00)|(0<<WGM00)|(0<<CS02)|(1<<CS01)|(1<<CS00);
	TIMSK |= (0<<OCIE0)|(1<<TOIE0);
}

void timer1_init(void) {
	// timer folosit pentru transmisia de date
	// mod 12 (CTC cu MAX stocat in ICR1)
	// frecventa ceas Timer1 = 15625Hz (prescaler = 1024)
	// overflow interrupt la fiecare 1s (1Hz)

	ICR1   = 0x3D08; // 15624
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<COM1C1)|(0<<COM1C0)|(1<<WGM11)|(0<<WGM10);
	TCNT1  = 0x00;
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(1<<CS12)|(0<<CS11)|(1<<CS10);
	TIMSK |= (0<<TICIE1)|(0<<OCIE1A)|(0<<OCIE1B)|(1<<TOIE1);
}

void timer2_init(void) {
	// timer dezactivat
	
	TCCR2 = (0<<FOC2)|(0<<WGM20)|(0<<COM21)|(0<<COM20)|(0<<WGM21)|(1<<CS22)|(0<<CS21)|(1<<CS20);
	TCNT2 = 0x00;
	OCR2  = 0x0F;
	TIMSK = (1<<OCIE2)|(1<<TOIE2);
}

void timer3_init(void) {
	// timer folosit pentru generarea semnalului PWM de 16kHz
	// mod 14 (Fast PWM cu MAX stocat in ICR3)
	// frecventa ceas Timer1 = 16kHz
	
	TCCR3A  = (0<<COM3A1)|(0<<COM3A0)|(0<<COM3B1)|(0<<COM3B0)|(0<<COM3C1)|(0<<COM3C0)|(1<<WGM31)|(0<<WGM30);
	ICR3	= 888; // frecventa = F_CPU/(prescaler*(1+ICR3)) = 18kHz => ICR3 = 888
	OCR3A   = 0x0000;
	OCR3B   = 0x0000;
	OCR3C   = 0x0000;
	TCNT3   = 0x0000;
	TCCR3B  = (0<<ICNC3)|(0<<ICES3)|(1<<WGM33)|(1<<WGM32)|(0<<CS32)|(0<<CS31)|(1<<CS30);
	ETIMSK |= (0<<TICIE3)|(0<<OCIE3A)|(0<<OCIE3B)|(0<<TOIE3)|(0<<OCIE3C)|(0<<OCIE1C);
}
