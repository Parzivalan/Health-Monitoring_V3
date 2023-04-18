#ifndef __LED_H
#define __LED_H

#include "sys.h"

#define LED_RED PAout(8)    //红灯-->PA8
#define LED_GREEN PAout(11) //绿灯-->PA11

#define LED_ON 1
#define LED_OFF 0

void LED_Init(void); //初始化

#endif /* __LED_H*/
