#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f4xx_hal.h"


HAL_StatusTypeDef LCD_Init(void);
void LCD_Clear(void);
void LCD_Print(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_PrintChar(char c);
HAL_StatusTypeDef I2C_SendCommand(uint8_t cmd);

#endif
