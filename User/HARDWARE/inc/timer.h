#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern u16 time_count;
extern u8 pow_f;
extern u8 shoot_f;
void Time3_Int_Init(void);
void Time4_Int_Init(void);

#endif /* __TIMER_H */
