/*
 * e-bike controller v1 / v2
 * file: fd_bldc.h
 *
 * Created: 10.04.2016
 * Author : Florin Dumitrache
 */

#ifndef BLDC_H
#define BLDC_H

#define PWMA_HIGH 3
#define PWMB_HIGH 4
#define PWMC_HIGH 5
#define PWMA_LOW  5
#define PWMB_LOW  6
#define PWMC_LOW  7

#define PORTB_reset_PWM_outputs (PORTB |= 0b11100000)
#define PORTE_reset_PWM_outputs (PORTE &= 0b11000111)

#define Timer3_OCR3x_disable        (TCCR3A  = (1<<WGM31))
#define Timer3_OCR3A_enable_nonInv  (TCCR3A |= (1<<COM3A1)|(0<<COM3A0))
#define Timer3_OCR3B_enable_nonInv  (TCCR3A |= (1<<COM3B1)|(0<<COM3B0))
#define Timer3_OCR3C_enable_nonInv  (TCCR3A |= (1<<COM3C1)|(0<<COM3C0))

#define deadtime_us 1

extern void BLDC_HPWM_LON(uint8_t hall, uint8_t enable, uint16_t duty);
extern void bldc_PWM_ON(uint8_t hall, uint8_t enable, uint16_t duty);
extern void bldc_shutdown(void);

#endif //BLDC_H