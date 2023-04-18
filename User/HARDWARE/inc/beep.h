#ifndef __BEEP_H
#define __BEEP_H

#include "sys.h"

#define BEEP PBout(1) //蜂鸣器-->PB1

void BEEP_Init(void); //初始化

#endif /* __BEEP_H */
