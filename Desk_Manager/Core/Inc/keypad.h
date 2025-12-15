#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32f4xx_hal.h"

void Keypad_Init(void);
char Keypad_Read(void);

#endif
