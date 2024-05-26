#ifndef SMD15_H
#define SMD15_H

#include "stm32f10x.h"



u8 countsum(u8 *buf);
void SMD15_init(u8 bound);
void start_scan(void);
void stop_scan(void);
void SMD15_setbaudrate(u8 i);
void SMD15_setScanfHZ(u8 hz);
void SMD15_setstandard(void);
void SMD15_setpixhawk(void);
void SDM15_Decode(uint8_t RxData);
u8 print_message(void);

#endif

