/*
 * e-bike controller v1 / v2
 * file: fd_timer.h
 *
 * Created: 31.03.2016
 * Author : Florin Dumitrache
 */

#ifndef TIMER_H
#define TIMER_H

#define timer1_on	TCCR1B = (1<<CS12)|(0<<CS11)|(1<<CS10)
#define timer1_off	TCCR1B = 0x00

extern void timer0_init(void); // functie pentru intializare timer0 (timer 8 biti)
extern void timer1_init(void); // functie pentru intializare timer1 (timer 16 biti)
extern void timer2_init(void); // functie pentru intializare timer2 (timer 8 biti)
extern void timer3_init(void); // functie pentru intializare timer3 (timer 16 biti)

#endif //TIMER_H