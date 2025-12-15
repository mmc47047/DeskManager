#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c1;

#define LCD_ADDR (0x27 << 1)
#define LCD_BACKLIGHT 0x08
#define ENABLE 0x04

HAL_StatusTypeDef I2C_SendCommand(uint8_t cmd);
void I2C_SendData(uint8_t data);
void LCD_Send(uint8_t data, uint8_t mode);
void LCD_Write4Bits(uint8_t value);

HAL_StatusTypeDef LCD_Init(void)
{
    HAL_Delay(50);
    LCD_Write4Bits(0x30);
    HAL_Delay(5);
    LCD_Write4Bits(0x30);
    HAL_Delay(5);
    LCD_Write4Bits(0x30);
    HAL_Delay(10);
    LCD_Write4Bits(0x20);

    if (I2C_SendCommand(0x28) != HAL_OK) return HAL_ERROR;
    if (I2C_SendCommand(0x08) != HAL_OK) return HAL_ERROR;
    if (I2C_SendCommand(0x01) != HAL_OK) return HAL_ERROR;
    HAL_Delay(5);
    if (I2C_SendCommand(0x06) != HAL_OK) return HAL_ERROR;
    if (I2C_SendCommand(0x0C) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

void LCD_Clear(void)
{
  I2C_SendCommand(0x01);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
  uint8_t row_offsets[] = {0x00, 0x40};
  I2C_SendCommand(0x80 | (col + row_offsets[row]));
}

void LCD_Print(char *str)
{
  while (*str)
    LCD_Send(*str++, 1);
}

void LCD_PrintChar(char c)
{
  LCD_Send(c, 1);
}

HAL_StatusTypeDef I2C_SendCommand(uint8_t cmd)
{
  // ??? ??? ?? RS=0 (mode=0)?? LCD_Send ??? ?????.
  LCD_Send(cmd, 0); 
  return HAL_OK;
}

void I2C_SendData(uint8_t data)
{
  LCD_Send(data, 1);
}

void LCD_Send(uint8_t data, uint8_t mode)
{
  uint8_t high = data & 0xF0;
  uint8_t low = (data << 4) & 0xF0;
  LCD_Write4Bits(high | mode | LCD_BACKLIGHT);
  LCD_Write4Bits(low | mode | LCD_BACKLIGHT);
}

void LCD_Write4Bits(uint8_t value)
{
  uint8_t data[1];
  data[0] = value | ENABLE;
  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data, 1, HAL_MAX_DELAY);
  data[0] = value & ~ENABLE;
  HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data, 1, HAL_MAX_DELAY);
}
