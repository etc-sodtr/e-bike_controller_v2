#ifndef ADC_H
#define ADC_H

/*
.----------------------------------------------------------------------.
|MUX4 | MUX3 | MUX2 | MUX1 | MUX0 | Intrare |          Functie         |
:----------------------------------------------------------------------:
| 0   |  0   |  0   |  0   |  0   |  ADC0   |   Temperatura MAX6175    |
| 0   |  0   |  0   |  0   |  1   |  ADC1   |       Curent motor       |
| 0   |  0   |  0   |  1   |  0   |  ADC2   |     Tensiune baterie     |
| 0   |  0   |  0   |  1   |  1   |  ADC3   |       ADC extern 3       |
| 0   |  0   |  1   |  0   |  0   |  ADC4   |       ADC extern 4       |
| 0   |  0   |  1   |  0   |  1   |  ADC5   |       ADC extern 5       |
| 0   |  0   |  1   |  1   |  0   |  ADC6   |       ADC extern 6       |
| 0   |  0   |  1   |  1   |  1   |  ADC7   |       Acceleratie        |
'----------------------------------------------------------------------'
*/

#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0))	// tensiunea de referinta pentru ADC este VREF = AREF

extern void ADC_init(void);						// functie pentru initializarea modulului ADC
extern uint8_t read_adc8(uint8_t adc_input);	// functie pentru citire ADC pe 10 biti
extern uint16_t read_adc10(uint8_t adc_input);	// functie pentru citire ADC pe 8 biti

#endif