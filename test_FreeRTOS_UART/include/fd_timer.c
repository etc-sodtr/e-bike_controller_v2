#include <avr/io.h>
#include "../include/fd_timer.h"

void timer2_init(void) {
	// timer dezactivat
	
	TCCR2 = (0<<FOC2)|(0<<WGM20)|(0<<COM21)|(0<<COM20)|(0<<WGM21)|(1<<CS22)|(0<<CS21)|(1<<CS20);
	TCNT2 = 0x00;
	OCR2  = 0x0F;
	TIMSK = (1<<OCIE2)|(1<<TOIE2);
}
