// output bit operations
#define PORTX(port)						(PORT ## port)
#define pin_SetHI(port,pin)				(PORTX(port) |= (1<<(pin)))
#define pin_SetLO(port,pin)				(PORTX(port) &= ~(1<<(pin)))
#define pin_Toggle(port,pin)			(PORTX(port) ^= (1<<(pin)))
#define pin_SetState(port,pin,state)	(state == 1) ? pin_SetHI(port,pin) : pin_SetLO(port,pin)

// input bit operations
#define PINX(port)						(PIN ## port)
#define PINY(port,pin)					(PIN ## port ## pin)
#define pin_ReadStateHI(port,pin)		((PINX(port) & (1<<(PINY(port,pin)))) == (1<<(PINY(port,pin))))
#define pin_ReadStateLO(port,pin)		((PINX(port) & (1<<(PINY(port,pin)))) == 0x00)
#define pin_ReadState(port,pin,state)	(state == 1) ? pin_ReadStateHI(port,pin) : pin_ReadStateLO(port,pin)